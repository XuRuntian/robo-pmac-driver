#include "continuum/disabled_probe.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    const continuum::DriveOffsets o{0, 2, 6, 8, 10, 12, 16, 18, 20};
    std::array<std::uint8_t, 24> data{};
    const std::array<std::int32_t, 5> positions{
        -506000555, 0, 506000555, std::numeric_limits<std::int32_t>::min(),
        std::numeric_limits<std::int32_t>::max()};
    for (const auto position : positions) {
        data.fill(0xff); // Even a stale all-ones image must become disabled.
        continuum::write_disabled_frame(data.data(), o, position);
        if (continuum::read_u16(data.data()) != 0 || continuum::read_i32(data.data() + 2) != position
                || continuum::read_u16(data.data() + 6) != 0 || data[8] != 8 || data[9] != 0)
            throw std::runtime_error("unsafe/incorrect disabled output frame");
        for (unsigned i = 10; i < data.size(); ++i)
            if (data[i] != 0xff) throw std::runtime_error("output overwrote feedback");
    }
    if (!continuum::disabled_status(0x0240) || !continuum::disabled_status(0x0250))
        throw std::runtime_error("drive-specific non-state bits rejected");
    for (const auto status : {0x0208, 0x0027, 0x0021, 0x0000, 0x000f})
        if (continuum::disabled_status(status))
            throw std::runtime_error("non-disabled CiA402 state admitted");
    std::cout << "PASS: disabled frame overwrites stale commands, preserves signed feedback and input bytes\n";
}
