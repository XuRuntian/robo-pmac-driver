#include "continuum/igh_config.hpp"
#include "continuum/offline_core.hpp"
#include "continuum/pdo.hpp"
#include "drive_config.hpp"
#include <dlfcn.h>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() try {
    // Enforce the offline library before requesting any master, even if the
    // process environment has overridden normal dynamic linking.
    Dl_info library{};
    check(dladdr(reinterpret_cast<void*>(&ecrt_request_master), &library)
          && library.dli_fname && std::string(library.dli_fname).find("libfakeethercat.so") != std::string::npos,
          "offline smoke test requires libfakeethercat; refusing real master access");
    check(std::getenv("FAKE_EC_HOMEDIR"), "set FAKE_EC_HOMEDIR to the offline working directory");
    check(ecrt_version_magic() == ECRT_VERSION_MAGIC, "IgH API mismatch");
    std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> master(ecrt_request_master(0), ecrt_release_master);
    check(bool(master), "fake master creation failed");
    auto* domain = ecrt_master_create_domain(master.get());
    check(domain, "fake domain creation failed");
    const auto offsets = continuum::configure_drives(master.get(), domain);
    check(ecrt_master_activate(master.get()) == 0, "fake activation failed");
    auto* data = ecrt_domain_data(domain);
    check(data, "missing fake process image");
    check(ecrt_domain_size(domain) == 5 * (10 + 14), "unexpected fake domain size");

    continuum::Limits limits;
    limits.lower.fill(-10000); limits.upper.fill(10000);
    limits.max_velocity.fill(200000); limits.max_acceleration.fill(20000000);
    continuum::Sample start;
    continuum::Segment end;
    end.ticks = 20; end.sequence = 1;
    // Distinct signs/counts detect swapped axes. These are SYNTHETIC counts.
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        start.position[i] = (i % 2 ? -1 : 1) * (1000.0 + 100 * i);
        end.position[i] = start.position[i] + 20 * (i + 1);
        const auto& o = offsets[i];
        check(o.target_position == o.control + 2 && o.target_torque == o.control + 6
              && o.mode == o.control + 8, "output PDO offsets wrong");
        check(o.actual_position == o.status + 2 && o.actual_torque == o.status + 6
              && o.mode_display == o.status + 8 && o.digital_inputs == o.status + 10,
              "input PDO offsets/padding wrong");
        if (i) check(o.control >= offsets[i-1].digital_inputs + 4, "overlapping fake slave domains");
    }
    continuum::Cubic cubic(start, end, continuum::drive_config::period_ns * 1e-9, limits);
    for (unsigned tick = 0; tick <= end.ticks; ++tick) {
        check(ecrt_master_receive(master.get()) == 0, "fake receive failed");
        ecrt_domain_process(domain);
        const auto sample = cubic.at(double(tick) / end.ticks);
        for (std::size_t i = 0; i < offsets.size(); ++i) {
            const auto& o = offsets[i];
            const auto counts = continuum::position_count(sample.position[i]);
            // Keep the hypothetical drives disabled throughout the smoke test.
            continuum::write_u16(data + o.control, 0);
            continuum::write_i32(data + o.target_position, counts);
            continuum::write_i16(data + o.target_torque, 0);
            EC_WRITE_S8(data + o.mode, continuum::drive_config::mode);
            // Inject a synthetic echo AFTER domain_process, which refreshes inputs.
            continuum::write_u16(data + o.status, 0x0250);
            continuum::write_i32(data + o.actual_position, counts);
            EC_WRITE_S8(data + o.mode_display, 8);
            continuum::write_u32(data + o.digital_inputs, 0x80000000U + i);
            check(continuum::read_i32(data + o.actual_position) == counts, "signed position mismatch");
            check(continuum::decode_status(continuum::read_u16(data + o.status)) == continuum::DriveState::disabled,
                  "status mask mismatch");
            check(continuum::read_u32(data + o.digital_inputs) == 0x80000000U + i, "digital input mismatch");
            check(data[o.mode + 1] == 0 && data[o.mode_display + 1] == 0, "padding overwritten");
        }
        ecrt_domain_queue(domain);
        check(ecrt_master_application_time(master.get(), 1000000000ULL + tick * continuum::drive_config::period_ns) == 0,
              "fake application time failed");
        check(ecrt_master_sync_reference_clock(master.get()) == 0, "fake reference sync failed");
        check(ecrt_master_sync_slave_clocks(master.get()) == 0, "fake slave sync failed");
        check(ecrt_master_send(master.get()) >= 0, "fake send failed");
    }
    std::cout << "PASS: official IgH fake library; five PDO configurations, 120-byte image, "
                 "21 synthetic PVT samples, all controlwords disabled\n";
    std::cout << "No physical drive, DC timing, WKC failure or motor dynamics was simulated by IgH.\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
}
