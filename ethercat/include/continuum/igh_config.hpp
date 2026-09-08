#pragma once
#include <ecrt.h>
#include <array>
#include <cstdint>

namespace continuum {
struct DriveOffsets {
    unsigned control, target_position, target_torque, mode;
    unsigned status, actual_position, actual_torque, mode_display, digital_inputs;
};
// Configuration API adapter; no master activation, send/receive or enable.
// For this phase only the explicitly fake-linked executable calls this.
std::array<DriveOffsets, 5> configure_drives(ec_master_t*, ec_domain_t*);
}
