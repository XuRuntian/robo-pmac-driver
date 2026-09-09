#pragma once
#include <ecrt.h>
#include <array>
#include <cstdint>

namespace continuum {
struct DriveOffsets {
    unsigned control, target_position, target_torque, mode;
    unsigned status, actual_position, actual_torque, mode_display, digital_inputs;
};
struct ConfiguredDrive { ec_slave_config_t* slave; DriveOffsets offsets; };
struct DriveTiming {
    unsigned period_ns = 1000000;
    unsigned sync1_register_ns = 0;
};
// Configuration only: no master activation or send/receive. Startup controlword
// is zero; a caller must separately validate actual topology before activation.
ConfiguredDrive configure_drive(ec_master_t*, ec_domain_t*, std::size_t index);
ConfiguredDrive configure_drive(ec_master_t*, ec_domain_t*, std::size_t index, DriveTiming);
std::array<DriveOffsets, 5> configure_drives(ec_master_t*, ec_domain_t*);
}
