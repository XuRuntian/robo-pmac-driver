#pragma once
#include "continuum/igh_config.hpp"
#include "continuum/pdo.hpp"

namespace continuum {
// This entry point has no enable/reset/trajectory argument. Every frame keeps
// power disabled, torque zero and target equal to a validated feedback sample.
inline void write_disabled_frame(std::uint8_t* data, const DriveOffsets& o,
                                 std::int32_t feedback) {
    write_u16(data + o.control, 0);
    write_i32(data + o.target_position, feedback);
    write_i16(data + o.target_torque, 0);
    EC_WRITE_S8(data + o.mode, 8);
    data[o.mode + 1] = 0;
}
inline bool disabled_status(std::uint16_t status) {
    return (status & 0x004f) == 0x0040;
}
} // namespace continuum
