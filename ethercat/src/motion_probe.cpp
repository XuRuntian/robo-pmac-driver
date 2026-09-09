// One unloaded Diamond, 2 ms CSP, bounded relative outward/return commissioning.
#include "continuum/commissioning_motion.hpp"
#include "continuum/cyclic_runtime.hpp"
#include "continuum/disabled_probe.hpp"
#include "drive_config.hpp"
#include <array>
#include <charconv>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

namespace {
constexpr std::uint64_t period = 2000000, second = 1000000000;
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void sleep_until(std::uint64_t deadline) {
    timespec t{static_cast<time_t>(deadline / second), static_cast<long>(deadline % second)};
    const auto r = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr);
    check(r == 0 || r == EINTR, "cyclic sleep failed");
}
std::uint32_t upload(ec_master_t* m, unsigned index, unsigned sub, unsigned bytes) {
    std::uint8_t data[4]{}; std::size_t size = 0; std::uint32_t abort = 0;
    check(ecrt_master_sdo_upload(m, 0, index, sub, data, sizeof data, &size, &abort) == 0
          && size == bytes, "preflight SDO failed");
    std::uint32_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) value |= std::uint32_t(data[i]) << (8 * i);
    return value;
}
void validate_profile(ec_master_t* m) {
    // This firmware reports fixed capacities at subindex zero, even after a
    // successful zero/count write. Require the entire expected prefix and ZERO
    // unused slots; fresh PDO -> SDO target echo is additionally required below.
    for (const auto assignment : {0x1c12, 0x1c13}) {
        check(upload(m, assignment, 0, 1) == 2
              && upload(m, assignment, 1, 2) == (assignment == 0x1c12 ? 0x1600U : 0x1a00U)
              && upload(m, assignment, 2, 2) == 0, "assignment differs from diagnosed Diamond profile");
    }
    for (const auto index : {0x1600, 0x1a00}) {
        const bool rx = index == 0x1600;
        const auto* entries = rx ? continuum::drive_config::rx_entries : continuum::drive_config::tx_entries;
        const unsigned used = rx ? 5 : 6, capacity = rx ? 10 : 12;
        check(upload(m, index, 0, 1) == capacity, "unknown PDO count behavior");
        for (unsigned sub = 1; sub <= capacity; ++sub) {
            const auto expected = sub <= used ? (std::uint32_t(entries[sub-1].index) << 16)
                | (unsigned(entries[sub-1].subindex) << 8) | entries[sub-1].bit_length : 0;
            check(upload(m, index, sub, 4) == expected, "PDO mapping/unused slot mismatch");
        }
    }
    char firmware[64]{}; std::size_t bytes = 0; std::uint32_t abort = 0;
    check(ecrt_master_sdo_upload(m, 0, 0x100a, 0, reinterpret_cast<std::uint8_t*>(firmware),
          sizeof firmware - 1, &bytes, &abort) == 0 && std::string(firmware) == "1.6.5.0.2.1.8.8",
          "firmware differs from diagnosed count behavior");
}
struct Echo { unsigned index, sub, bytes; ec_sdo_request_t* request; std::uint32_t value = 0; };
struct Sample { std::uint64_t time; int phase; std::int32_t position, target; int torque; unsigned status, control, dc; };
}

int main(int argc, char** argv) {
    bool motion_requested = false, activated = false, enabled_sent = false, echo_verified = false;
    bool completed = false, disabled_confirmed = false;
    int displacement = 90, move_ms = 2000, host_bound = 256, drive_bound = 512;
    unsigned op_cycles = 0, bad_wkc = 0, bad_op = 0, dc_max_motion = 0;
    std::uint64_t max_late = 0, qualified_after = 0;
    int final_phase = -1, policy_failure = 0;
    std::string failure;
    std::int32_t initial = 0, position = 0, target = 0, minimum = 0, maximum = 0;
    unsigned status = 0; int torque = 0;
    std::vector<Sample> samples; samples.reserve(12000);
    std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> master(nullptr, ecrt_release_master);
    ec_domain_t* domain = nullptr; std::uint8_t* data = nullptr;
    continuum::ConfiguredDrive configured{};
    continuum::CyclicRuntime runtime;
    try {
        check(argc >= 2, "specify --verify-disabled or --move-unloaded-operator-ready");
        const bool legacy = std::string(argv[1]) == "--move-unloaded-90-counts-operator-ready";
        motion_requested = legacy || std::string(argv[1]) == "--move-unloaded-operator-ready";
        check(motion_requested || std::string(argv[1]) == "--verify-disabled", "unknown motion option");
        check(!legacy || argc == 2, "legacy entry remains fixed at 90 counts / 2 seconds");
        bool have_counts = false, have_time = false;
        for (int i = 2; i < argc; ++i) {
            const std::string key = argv[i];
            check(++i < argc, "option requires a value");
            const std::string value = argv[i]; int parsed = 0;
            const auto r = std::from_chars(value.data(), value.data()+value.size(), parsed);
            check(r.ec == std::errc() && r.ptr == value.data()+value.size(), "invalid integer option");
            if (key == "--counts" && !have_counts) { displacement = parsed; have_counts = true; }
            else if (key == "--move-ms" && !have_time) { move_ms = parsed; have_time = true; }
            else throw std::runtime_error("unknown or repeated motion option");
        }
        check(displacement && std::abs(std::int64_t(displacement)) <= 3641,
              "displacement must be nonzero and within +/-3641 counts (about 10 degrees)");
        check(move_ms >= 1000 && move_ms <= 5000, "one-way duration must be 1000-5000 ms");
        host_bound = std::max(256, std::abs(displacement)+128);
        drive_bound = host_bound+256;
#if defined(__SANITIZE_ADDRESS__)
        throw std::runtime_error("hardware commissioning requires the Release build");
#endif
        check(geteuid() == 0 && ecrt_version_magic() == ECRT_VERSION_MAGIC, "root/API check failed");
        std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
        master.reset(ecrt_request_master(0)); check(bool(master), "master request failed");
        ec_master_info_t info{}; ec_slave_info_t drive{};
        const auto& id = continuum::drive_config::slaves[0];
        check(ecrt_master(master.get(), &info) == 0 && info.slave_count == 1 && info.link_up && !info.scan_busy,
              "requires one linked slave");
        check(ecrt_master_get_slave(master.get(), 0, &drive) == 0 && drive.vendor_id == id.vendor
              && drive.product_code == id.product && drive.revision_number == id.revision && drive.alias == 0
              && drive.al_state == 2 && !drive.error_flag, "identity/PREOP check failed");
        check(upload(master.get(), 0x6040, 0, 2) == 0
              && continuum::disabled_status(upload(master.get(), 0x6041, 0, 2))
              && upload(master.get(), 0x603f, 0, 2) == 0, "requires healthy disabled drive");
        validate_profile(master.get());
        const auto raw = upload(master.get(), 0x6064, 0, 4);
        std::memcpy(&initial, &raw, sizeof raw); position = target = minimum = maximum = initial;
        check(std::int64_t(initial) - drive_bound >= INT32_MIN && std::int64_t(initial) + drive_bound <= INT32_MAX,
              "position too close to integer boundary");
        for (const auto index : {0x6072, 0x60e0, 0x60e1})
            check(upload(master.get(), index, 0, 2) == 30, "3 percent drive torque limits required");
        check(upload(master.get(), 0x6065, 0, 4) == 128 && upload(master.get(), 0x6066, 0, 2) == 20,
              "drive following error limits required");
        const auto low = static_cast<std::int32_t>(upload(master.get(), 0x607d, 1, 4));
        const auto high = static_cast<std::int32_t>(upload(master.get(), 0x607d, 2, 4));
        check(std::abs(std::int64_t(initial)-low-drive_bound) <= 64
              && std::abs(std::int64_t(high)-initial-drive_bound) <= 64,
              "narrow drive position limits required");
        domain = ecrt_master_create_domain(master.get()); check(domain, "domain allocation failed");
        configured = continuum::configure_drive(master.get(), domain, 0, {period, 0});
        std::array<Echo, 6> echoes{{{0x607a,0,4,nullptr}, {0x6040,0,2,nullptr}, {0x6071,0,2,nullptr},
                                  {0x6060,0,1,nullptr}, {0x6064,0,4,nullptr}, {0x603f,0,2,nullptr}}};
        for (auto& e : echoes) {
            e.request = ecrt_slave_config_create_sdo_request(configured.slave, e.index, e.sub, e.bytes);
            check(e.request && ecrt_sdo_request_timeout(e.request, 1000) == 0, "echo request allocation failed");
        }
        check(ecrt_domain_size(domain) == 24, "unexpected domain length");
        check(ecrt_master_set_send_interval(master.get(), 2000) == 0, "send interval failed");
        check(ecrt_master_activate(master.get()) == 0, "activation failed"); activated = true;
        data = ecrt_domain_data(domain); check(data, "missing process image");
        continuum::write_disabled_frame(data, configured.offsets, target);
        runtime.start(true, 2);
        const auto start = continuum::monotonic_ns(); auto next = start, first_op = std::uint64_t(0);
        std::uint64_t proof_start = 0, stage_start = 0;
        continuum::StableWindow settled(3 * second);
        continuum::CommissioningMotion motion({displacement,host_bound,64,100,period,
                                               std::uint64_t(move_ms)*1000000});
        unsigned echo_stage = 0, echo_cursor = 0;
        bool request_started = false;
        while (!completed) {
            next += period; sleep_until(next); const auto current = continuum::monotonic_ns();
            const auto late = current > next ? current-next : 0; max_late = std::max(max_late, late);
            check(!stopping, "operator signal received");
            check(late < period, "missed cyclic deadline");
            check(ecrt_master_application_time(master.get(), next) == 0 && ecrt_master_receive(master.get()) == 0,
                  "cyclic receive/application time failed");
            ecrt_domain_process(domain);
            ec_domain_state_t ds{}; ec_slave_config_state_t ss{}; ec_master_state_t ms{};
            check(ecrt_domain_state(domain, &ds) == 0 && ecrt_slave_config_state(configured.slave, &ss) == 0
                  && ecrt_master_state(master.get(), &ms) == 0, "cyclic state read failed");
            const bool wkc = ds.wc_state == EC_WC_COMPLETE && ds.working_counter == 3;
            const bool op = ss.online && ss.operational && ss.al_state == 8;
            check(ms.link_up && ms.slaves_responding == 1, "link/topology changed");
            if (wkc && op && !first_op) first_op = current;
            unsigned control = 0, dc = UINT32_MAX;
            if (first_op) {
                ++op_cycles; bad_wkc += !wkc; bad_op += !op;
                check(wkc && op, "OP or WKC lost");
                const auto& o = configured.offsets;
                position = continuum::read_i32(data + o.actual_position);
                status = continuum::read_u16(data + o.status); torque = EC_READ_S16(data + o.actual_torque);
                minimum = std::min(minimum, position); maximum = std::max(maximum, position);
                check(EC_READ_S8(data + o.mode_display) == 8, "unexpected mode");
                check(std::abs(std::int64_t(position)-initial) <= (enabled_sent ? host_bound : 64)
                      && std::abs(torque) <= 100,
                      "feedback position/torque exceeded bound");
                dc = ecrt_master_sync_monitor_process(master.get());
                if (!proof_start && settled.observe(current, dc <= 20000)) {
                    proof_start = stage_start = current; qualified_after = current-first_op;
                    target = initial + 13;
                }
                if (!echo_verified) {
                    check(continuum::disabled_status(status), "drive enabled during disabled echo test");
                    if (proof_start && current-stage_start >= 50000000) {
                        auto& e = echoes[echo_cursor];
                        if (!request_started) {
                            check(ecrt_sdo_request_read(e.request) == 0, "echo request failed"); request_started = true;
                        } else {
                            const auto state = ecrt_sdo_request_state(e.request);
                            check(state == EC_REQUEST_BUSY || state == EC_REQUEST_SUCCESS, "echo SDO failed");
                            if (state == EC_REQUEST_SUCCESS) {
                                check(ecrt_sdo_request_data_size(e.request) == e.bytes, "echo SDO size mismatch");
                                e.value = 0; const auto* ptr = ecrt_sdo_request_data(e.request);
                                for (unsigned i=0;i<e.bytes;++i) e.value |= std::uint32_t(ptr[i]) << (8*i);
                                if (echo_cursor == 0) check(e.value == static_cast<std::uint32_t>(target), "PDO target echo mismatch");
                                if (echo_cursor == 1 || echo_cursor == 2 || echo_cursor == 5) check(e.value == 0, "PDO zero/error echo mismatch");
                                if (echo_cursor == 3) check(e.value == 8, "PDO mode echo mismatch");
                                if (echo_cursor == 4) check(std::abs(std::int64_t(static_cast<std::int32_t>(e.value))-position) <= 16,
                                                           "PDO position/SDO feedback mismatch");
                                request_started = false;
                                if (++echo_cursor == echoes.size()) {
                                    echo_cursor = 0; stage_start = current;
                                    if (++echo_stage == 3) { echo_verified = true; target = position; }
                                    else target = initial + (echo_stage == 1 ? -17 : 0);
                                }
                            }
                        }
                    }
                } else if (motion_requested) {
                    check(dc <= 100000, "DC exceeded 100 us during motion");
                    dc_max_motion = std::max(dc_max_motion, dc);
                    const auto cmd = motion.step({current,static_cast<std::uint16_t>(status),position,
                        static_cast<std::int16_t>(torque),8,wkc,bool(ms.link_up),bool(stopping)});
                    target = cmd.position; control = cmd.control;
                    final_phase = static_cast<int>(motion.phase()); policy_failure = static_cast<int>(motion.failure());
                    check(motion.phase() != continuum::CommissioningMotion::Phase::aborted, "bounded motion policy aborted");
                    completed = motion.phase() == continuum::CommissioningMotion::Phase::done;
                    check(samples.size() < samples.capacity(), "motion record capacity reached");
                    samples.push_back({current-proof_start,final_phase,position,target,torque,status,control,dc});
                } else completed = true;
            }
            continuum::write_disabled_frame(data, configured.offsets, target);
            continuum::write_u16(data + configured.offsets.control, control);
            ecrt_domain_queue(domain);
            const auto send_time = continuum::monotonic_ns();
            check(ecrt_master_sync_reference_clock_to(master.get(), send_time) == 0
                  && ecrt_master_sync_slave_clocks(master.get()) == 0 && ecrt_master_sync_monitor_queue(master.get()) == 0,
                  "clock synchronization failed");
            if (control == 15) enabled_sent = true; // Conservative: send may partially succeed.
            check(ecrt_master_send(master.get()) >= 0, "cyclic send failed");
            check(first_op || current-start < 20*second, "OP deadline exceeded");
            check(!first_op || proof_start || current-first_op < 45*second, "DC qualification deadline exceeded");
            check(current-start < 110*second, "commissioning deadline exceeded");
        }
    } catch (const std::exception& e) { failure = e.what(); }

    // Every normal/exception/signal exit attempts disable, continuing cyclic
    // transport for one second. Lost communication can never confirm stopping.
    if (activated && data) {
        try {
            auto next = continuum::monotonic_ns(); const auto end = next + second;
            continuum::StableWindow disabled(200000000);
            while (next < end) {
                next += period; sleep_until(next);
                check(ecrt_master_application_time(master.get(), next) == 0 && ecrt_master_receive(master.get()) == 0,
                      "disable receive failed");
                ecrt_domain_process(domain); ec_domain_state_t ds{}; ec_slave_config_state_t ss{};
                check(ecrt_domain_state(domain, &ds) == 0 && ecrt_slave_config_state(configured.slave, &ss) == 0,
                      "disable state failed");
                bool valid = ds.wc_state == EC_WC_COMPLETE && ds.working_counter == 3 && ss.operational;
                if (valid) {
                    position = continuum::read_i32(data + configured.offsets.actual_position);
                    status = continuum::read_u16(data + configured.offsets.status);
                    valid = continuum::disabled_status(status);
                }
                disabled_confirmed = disabled.observe(continuum::monotonic_ns(), valid);
                continuum::write_disabled_frame(data, configured.offsets, position); ecrt_domain_queue(domain);
                check(ecrt_master_sync_reference_clock_to(master.get(), continuum::monotonic_ns()) == 0
                      && ecrt_master_sync_slave_clocks(master.get()) == 0 && ecrt_master_send(master.get()) >= 0,
                      "disable send failed");
            }
        } catch (const std::exception& e) { disabled_confirmed = false; if (failure.empty()) failure = e.what(); }
    }
    runtime.restore(); master.reset();
    if (!disabled_confirmed && failure.empty()) failure = "final disabled feedback unconfirmed";
    if (!failure.empty()) std::cerr << "Commissioning stopped: " << failure << '\n';
    std::cout << "{\"motion_requested\":" << motion_requested << ",\"activated\":" << activated
        << ",\"displacement_counts\":" << displacement << ",\"move_ms\":" << move_ms
        << ",\"host_travel_bound_counts\":" << host_bound << ",\"drive_travel_bound_counts\":" << drive_bound
        << ",\"enable_commands_sent\":" << enabled_sent << ",\"echo_verified\":" << echo_verified
        << ",\"completed\":" << completed << ",\"disabled_confirmed\":" << disabled_confirmed
        << ",\"op_cycles\":" << op_cycles << ",\"bad_wkc\":" << bad_wkc << ",\"bad_op\":" << bad_op
        << ",\"max_lateness_ns\":" << max_late << ",\"qualified_after_op_ns\":" << qualified_after
        << ",\"dc_max_motion_ns\":" << dc_max_motion << ",\"final_phase\":" << final_phase
        << ",\"policy_failure\":" << policy_failure << ",\"initial_position\":" << initial
        << ",\"final_position\":" << position << ",\"minimum_position\":" << minimum
        << ",\"maximum_position\":" << maximum << ",\"final_statusword\":" << status << ",\"samples\":[";
    bool comma = false;
    for (const auto& s : samples) {
        if (comma) std::cout << ',';
        comma = true;
        std::cout << '[' << s.time << ',' << s.phase << ',' << s.position << ',' << s.target << ','
                  << s.torque << ',' << s.status << ',' << s.control << ',' << s.dc << ']';
    }
    std::cout << "]}\n";
    return failure.empty() && completed && disabled_confirmed ? 0 : 1;
}
