#pragma once
#include <ecrt.h>
#include <cstdint>
#include <cstring>

namespace continuum {
// Use IgH's byte order conversion on aligned temporaries. Several 32-bit
// entries are at byte offset 2, so dereferencing uint32_t* in-place is unsafe.
inline void write_u16(std::uint8_t* dst, std::uint16_t value) {
    std::uint16_t raw;
    EC_WRITE_U16(&raw, value);
    std::memcpy(dst, &raw, sizeof raw);
}
inline void write_i16(std::uint8_t* dst, std::int16_t value) {
    write_u16(dst, static_cast<std::uint16_t>(value));
}
inline void write_u32(std::uint8_t* dst, std::uint32_t value) {
    std::uint32_t raw;
    EC_WRITE_U32(&raw, value);
    std::memcpy(dst, &raw, sizeof raw);
}
inline void write_i32(std::uint8_t* dst, std::int32_t value) {
    write_u32(dst, static_cast<std::uint32_t>(value));
}
inline std::uint16_t read_u16(const std::uint8_t* src) {
    std::uint16_t raw;
    std::memcpy(&raw, src, sizeof raw);
    return EC_READ_U16(&raw);
}
inline std::int32_t read_i32(const std::uint8_t* src) {
    std::uint32_t raw;
    std::memcpy(&raw, src, sizeof raw);
    return EC_READ_S32(&raw);
}
inline std::uint32_t read_u32(const std::uint8_t* src) {
    std::uint32_t raw;
    std::memcpy(&raw, src, sizeof raw);
    return EC_READ_U32(&raw);
}
} // namespace continuum
