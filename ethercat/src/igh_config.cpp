#include "continuum/igh_config.hpp"
#include "drive_config.hpp"
#include "esi_config.hpp"
#include <stdexcept>

namespace continuum {
ConfiguredDrive configure_drive(ec_master_t* master, ec_domain_t* domain, std::size_t i) {
    return configure_drive(master, domain, i,
                           {drive_config::period_ns, drive_config::sync1_register_ns});
}
ConfiguredDrive configure_drive(ec_master_t* master, ec_domain_t* domain, std::size_t i, DriveTiming timing) {
    if (!master || !domain) throw std::invalid_argument("null IgH master/domain");
    if (timing.period_ns % 1000000 || timing.period_ns < 1000000 || timing.period_ns > 4000000
            || (timing.sync1_register_ns != 0 && timing.sync1_register_ns != 500000))
        throw std::invalid_argument("unsupported Diamond commissioning timing");
    const auto& id = drive_config::slaves.at(i);
    auto* slave = ecrt_master_slave_config(master, id.alias, id.position, id.vendor, id.product);
    if (!slave) throw std::runtime_error("slave configuration failed");
    if (ecrt_slave_config_pdos(slave, EC_END, drive_config::syncs))
        throw std::runtime_error("PDO configuration failed");
    if (ecrt_slave_config_dc(slave, drive_config::assign_activate,
            timing.period_ns, drive_config::sync0_shift_ns, timing.sync1_register_ns, 0))
        throw std::runtime_error("DC configuration failed");
    if (id.reference && ecrt_master_select_reference_clock(master, slave))
        throw std::runtime_error("reference clock selection failed");
    // ESI startup says CSP/2 ms. Retain PMAC's 1 ms application/DC period,
    // explicitly matching 60C2 instead of relying on the drive's last value.
    static_assert(esi_config::period_ms * 1000000 == drive_config::period_ns);
    if (ecrt_slave_config_sdo16(slave, 0x6040, 0, 0)
            || ecrt_slave_config_sdo8(slave, 0x6060, 0, drive_config::mode)
            || ecrt_slave_config_sdo8(slave, 0x60c2, 1, timing.period_ns / 1000000)
            || ecrt_slave_config_sdo8(slave, 0x60c2, 2, esi_config::time_index_byte))
        throw std::runtime_error("disabled CSP/interpolation SDO configuration failed");
    auto reg = [&](std::uint16_t index) -> unsigned {
        unsigned bit = 0;
        const auto offset = ecrt_slave_config_reg_pdo_entry(slave, index, 0, domain, &bit);
        if (offset < 0 || bit != 0) throw std::runtime_error("PDO registration/alignment failed");
        return static_cast<unsigned>(offset);
    };
    return {slave, {reg(0x6040), reg(0x607a), reg(0x6071), reg(0x6060),
                    reg(0x6041), reg(0x6064), reg(0x6077), reg(0x6061), reg(0x60fd)}};
}

std::array<DriveOffsets, 5> configure_drives(ec_master_t* master, ec_domain_t* domain) {
    std::array<DriveOffsets, 5> offsets{};
    for (std::size_t i = 0; i < offsets.size(); ++i)
        offsets[i] = configure_drive(master, domain, i).offsets;
    return offsets;
}
}
