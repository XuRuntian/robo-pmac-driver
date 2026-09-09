// Bounded single-drive commissioning, never a motor enable or motion entry point.
#include "continuum/disabled_probe.hpp"
#include "drive_config.hpp"
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
std::uint64_t now(clockid_t clock) {
    timespec t{};
    check(clock_gettime(clock, &t) == 0, "clock_gettime failed");
    return std::uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
void sleep_until(std::uint64_t deadline) {
    const timespec t{static_cast<time_t>(deadline / 1000000000ULL),
                     static_cast<long>(deadline % 1000000000ULL)};
    int result;
    do { result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr); }
    while (result == EINTR && !stopping);
    check(result == 0 || result == EINTR, "clock_nanosleep failed");
}
std::uint32_t upload(ec_master_t* master, std::uint16_t index, unsigned bytes) {
    std::uint8_t data[4]{};
    std::size_t size = 0;
    std::uint32_t abort = 0;
    check(ecrt_master_sdo_upload(master, 0, index, 0, data, sizeof data, &size, &abort) == 0
          && size == bytes, "preflight SDO upload failed");
    if (bytes == 2) return continuum::read_u16(data);
    return continuum::read_u32(data);
}
struct Readback {
    std::uint16_t index;
    std::uint8_t subindex;
    unsigned bytes;
    ec_sdo_request_t* request;
    bool complete = false;
    std::uint32_t value = 0;
};
struct Result {
    bool activated = false, reached_op = false, observation_complete = false;
    unsigned cycles = 0, op_cycles = 0, incomplete_cycles = 0, non_op_cycles = 0;
    unsigned missed_periods = 0, dc_samples = 0, dc_missing = 0;
    unsigned wkc_min = std::numeric_limits<unsigned>::max(), wkc_max = 0, dc_max_ns = 0;
    std::uint64_t max_lateness_ns = 0;
    std::uint16_t last_status = 0;
    int last_mode = 0;
    std::int32_t initial_position = 0, last_position = 0, min_position = 0, max_position = 0;
    std::vector<Readback> active_sdos;
    void print() const {
        std::cout << "{\"activated\":" << activated << ",\"reached_op\":" << reached_op
            << ",\"observation_complete\":" << observation_complete
            << ",\"controlword_always_zero\":true,\"enable_commands_sent\":false"
            << ",\"application_period_ns\":" << continuum::drive_config::period_ns
            << ",\"cycles\":" << cycles << ",\"op_cycles\":" << op_cycles
            << ",\"incomplete_cycles_after_op\":" << incomplete_cycles
            << ",\"non_op_cycles_after_op\":" << non_op_cycles
            << ",\"missed_periods\":" << missed_periods
            << ",\"max_lateness_ns\":" << max_lateness_ns
            << ",\"wkc_min_after_op\":" << (reached_op ? wkc_min : 0)
            << ",\"wkc_max_after_op\":" << wkc_max
            << ",\"dc_samples_after_op\":" << dc_samples
            << ",\"dc_missing_after_op\":" << dc_missing
            << ",\"dc_max_difference_ns_after_op\":" << dc_max_ns
            << ",\"last_statusword\":" << last_status << ",\"last_mode_display\":" << last_mode
            << ",\"initial_position\":" << initial_position << ",\"last_position\":" << last_position
            << ",\"minimum_position\":" << min_position << ",\"maximum_position\":" << max_position
            << ",\"active_sdo_readback\":[";
        bool comma = false;
        for (const auto& r : active_sdos) {
            if (comma) std::cout << ',';
            comma = true;
            std::cout << "{\"index\":" << r.index << ",\"subindex\":" << unsigned(r.subindex)
                      << ",\"bytes\":" << r.bytes << ",\"complete\":" << r.complete
                      << ",\"value\":" << r.value << '}';
        }
        std::cout << "]}\n";
    }
};
}

int main(int argc, char** argv) {
    Result result;
    int exit_code = 1;
    try {
        check(argc == 2 && std::string(argv[1]) == "--disabled-single-axis",
              "use scripts/probe_disabled.py; this binary accepts only --disabled-single-axis");
        check(geteuid() == 0, "root commissioning wrapper required");
        check(ecrt_version_magic() == ECRT_VERSION_MAGIC, "IgH API mismatch");
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> master(ecrt_request_master(0), ecrt_release_master);
        check(bool(master), "master request failed");
        ec_master_info_t info{};
        check(ecrt_master(master.get(), &info) == 0 && info.slave_count == 1 && info.link_up
              && !info.scan_busy, "expected one stable linked slave");
        ec_slave_info_t drive{};
        const auto& id = continuum::drive_config::slaves[0];
        check(ecrt_master_get_slave(master.get(), 0, &drive) == 0
              && drive.vendor_id == id.vendor && drive.product_code == id.product
              && drive.revision_number == id.revision && drive.alias == 0
              && drive.al_state == 2 && !drive.error_flag, "identity/PREOP check failed");
        check(upload(master.get(), 0x6040, 2) == 0, "existing controlword is not zero");
        check(continuum::disabled_status(upload(master.get(), 0x6041, 2)), "drive is not disabled");
        check(upload(master.get(), 0x603f, 2) == 0, "drive has a fault code");
        const auto raw_position = upload(master.get(), 0x6064, 4);
        std::memcpy(&result.initial_position, &raw_position, sizeof raw_position);
        result.last_position = result.min_position = result.max_position = result.initial_position;
        auto target = result.initial_position;
        auto* domain = ecrt_master_create_domain(master.get());
        check(domain, "domain creation failed");
        const auto configured = continuum::configure_drive(master.get(), domain, 0);
        const auto& o = configured.offsets;
        auto add_read = [&](std::uint16_t index, unsigned sub, unsigned bytes) {
            auto* request = ecrt_slave_config_create_sdo_request(configured.slave, index, sub, bytes);
            check(request && ecrt_sdo_request_timeout(request, 1000) == 0, "async read allocation failed");
            result.active_sdos.push_back({index, static_cast<std::uint8_t>(sub), bytes, request});
        };
        for (const auto index : {0x1c12, 0x1c13}) {
            add_read(index, 0, 1);
            add_read(index, 1, 2);
            add_read(index, 2, 2);
        }
        for (const auto index : {0x1600, 0x1a00}) {
            add_read(index, 0, 1);
            for (unsigned sub = 1; sub <= (index == 0x1600 ? 10U : 12U); ++sub)
                add_read(index, sub, 4);
        }
        add_read(0x60c2, 1, 1);
        add_read(0x60c2, 2, 1);
        add_read(0x6040, 0, 2);
        add_read(0x603f, 0, 2);
        std::size_t read_cursor = 0;
        check(ecrt_domain_size(domain) == 24, "single-drive PDO domain must be 24 bytes");
        const auto epoch_offset = now(CLOCK_REALTIME) - now(CLOCK_MONOTONIC) - 946684800000000000ULL;
        check(ecrt_master_application_time(master.get(), epoch_offset + now(CLOCK_MONOTONIC)) == 0,
              "initial application time failed");
        check(ecrt_master_activate(master.get()) == 0, "master activation failed");
        result.activated = true;
        auto* data = ecrt_domain_data(domain);
        check(data, "missing process image");
        continuum::write_disabled_frame(data, o, target);
        const auto start = now(CLOCK_MONOTONIC);
        auto next = start, last_good = start, first_op = std::uint64_t(0), last_log = start;
        constexpr auto period = continuum::drive_config::period_ns;
        // At most 20 s to reach OP, then 10 s observation. No catch-up burst.
        while (!stopping) {
            next += period;
            sleep_until(next);
            const auto current = now(CLOCK_MONOTONIC);
            const auto late = current > next ? current - next : 0;
            result.max_lateness_ns = std::max(result.max_lateness_ns, late);
            result.missed_periods += late / period;
            if (late >= period) next += (late / period) * period;
            check(ecrt_master_application_time(master.get(), epoch_offset + current) == 0, "application time failed");
            check(ecrt_master_receive(master.get()) == 0, "receive failed");
            ecrt_domain_process(domain);
            ec_domain_state_t domain_state{};
            ec_slave_config_state_t slave_state{};
            ec_master_state_t master_state{};
            ecrt_domain_state(domain, &domain_state);
            ecrt_slave_config_state(configured.slave, &slave_state);
            check(ecrt_master_state(master.get(), &master_state) == 0, "master state failed");
            const bool complete = domain_state.wc_state == EC_WC_COMPLETE;
            const bool operational = slave_state.online && slave_state.operational && slave_state.al_state == 8;
            const bool good = complete && operational;
            bool unexpected_drive = false;
            if (complete) {
                result.last_status = continuum::read_u16(data + o.status);
                result.last_mode = EC_READ_S8(data + o.mode_display);
                result.last_position = continuum::read_i32(data + o.actual_position);
                unexpected_drive = !continuum::disabled_status(result.last_status);
                if (!unexpected_drive) target = result.last_position;
                result.min_position = std::min(result.min_position, result.last_position);
                result.max_position = std::max(result.max_position, result.last_position);
            }
            if (good && !first_op) {
                first_op = current;
                result.reached_op = true;
                std::cerr << "Single drive reached OP with complete WKC; observing disabled feedback for 10 s.\n";
            }
            if (good) last_good = current;
            if (first_op) {
                ++result.op_cycles;
                result.incomplete_cycles += !complete;
                result.non_op_cycles += !operational;
                result.wkc_min = std::min(result.wkc_min, domain_state.working_counter);
                result.wkc_max = std::max(result.wkc_max, domain_state.working_counter);
                const auto dc = ecrt_master_sync_monitor_process(master.get());
                if (dc == std::numeric_limits<std::uint32_t>::max()) ++result.dc_missing;
                else { ++result.dc_samples; result.dc_max_ns = std::max(result.dc_max_ns, dc); }
                // Only one mailbox read in flight, serviced by this cycle's
                // send/receive. Never block the cyclic thread on CLI/SDO I/O.
                if (read_cursor < result.active_sdos.size()) {
                    auto& r = result.active_sdos[read_cursor];
                    switch (ecrt_sdo_request_state(r.request)) {
                    case EC_REQUEST_UNUSED:
                        check(ecrt_sdo_request_read(r.request) == 0, "async read scheduling failed");
                        break;
                    case EC_REQUEST_BUSY: break;
                    case EC_REQUEST_SUCCESS: {
                        check(ecrt_sdo_request_data_size(r.request) == r.bytes, "async read size mismatch");
                        const auto* bytes = ecrt_sdo_request_data(r.request);
                        check(bytes, "async read data missing");
                        for (unsigned i = 0; i < r.bytes; ++i) r.value |= std::uint32_t(bytes[i]) << (8 * i);
                        r.complete = true;
                        ++read_cursor;
                        break;
                    }
                    default: throw std::runtime_error("active SDO read failed; no automatic retry");
                    }
                }
            }
            // All paths, including an unexpected state, transmit only disable.
            continuum::write_disabled_frame(data, o, target);
            ecrt_domain_queue(domain);
            check(ecrt_master_sync_reference_clock(master.get()) == 0, "reference clock sync failed");
            check(ecrt_master_sync_slave_clocks(master.get()) == 0, "slave clock sync failed");
            check(ecrt_master_sync_monitor_queue(master.get()) == 0, "DC monitor queue failed");
            check(ecrt_master_send(master.get()) >= 0, "send failed");
            ++result.cycles;
            check(!unexpected_drive, "feedback left Switch On Disabled; probe stopped");
            check(master_state.link_up && master_state.slaves_responding == 1, "link/topology changed");
            check(!good || result.last_mode == 8, "mode display differs from CSP");
            check(!first_op || current - last_good < 100000000ULL, "cyclic feedback absent for 100 ms");
            check(first_op || current - start < 20000000000ULL, "did not reach OP within 20 s");
            if (current - last_log >= 1000000000ULL) {
                std::cerr << "AL=" << slave_state.al_state << " WKC=" << domain_state.working_counter
                          << " status=" << result.last_status << " position=" << result.last_position << '\n';
                last_log = current;
            }
            if (first_op && current - first_op >= 10000000000ULL) {
                check(read_cursor == result.active_sdos.size(), "active SDO readback incomplete");
                result.observation_complete = true;
                break;
            }
        }
        check(!stopping, "interrupted; master released with all sent controlwords zero");
        exit_code = 0;
    } catch (const std::exception& error) {
        std::cerr << "Disabled probe stopped: " << error.what() << '\n';
    }
    result.print();
    return exit_code;
}
