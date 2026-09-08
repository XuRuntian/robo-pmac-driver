#include "continuum/igh_config.hpp"
#include "drive_config.hpp"
#include <stdexcept>

namespace continuum {
std::array<DriveOffsets, 5> configure_drives(ec_master_t* master, ec_domain_t* domain) {
    if (!master || !domain) throw std::invalid_argument("null IgH master/domain");
    std::array<DriveOffsets, 5> offsets{};
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        const auto& id = drive_config::slaves[i];
        auto* slave = ecrt_master_slave_config(master, id.alias, id.position, id.vendor, id.product);
        if (!slave) throw std::runtime_error("slave configuration failed");
        if (ecrt_slave_config_pdos(slave, EC_END, drive_config::syncs))
            throw std::runtime_error("PDO configuration failed");
        if (ecrt_slave_config_dc(slave, drive_config::assign_activate,
                drive_config::sync0_cycle_ns, drive_config::sync0_shift_ns,
                drive_config::sync1_register_ns, 0))
            throw std::runtime_error("DC configuration failed");
        if (id.reference && ecrt_master_select_reference_clock(master, slave))
            throw std::runtime_error("reference clock selection failed");
        // Match PMAC's requested CSP mode. No drive enable is requested here.
        if (ecrt_slave_config_sdo8(slave, 0x6060, 0, drive_config::mode))
            throw std::runtime_error("mode SDO configuration failed");
        auto reg = [&](std::uint16_t index) -> unsigned {
            unsigned bit = 0;
            const auto offset = ecrt_slave_config_reg_pdo_entry(slave, index, 0, domain, &bit);
            if (offset < 0 || bit != 0) throw std::runtime_error("PDO registration/alignment failed");
            return static_cast<unsigned>(offset);
        };
        offsets[i] = {reg(0x6040), reg(0x607a), reg(0x6071), reg(0x6060),
                      reg(0x6041), reg(0x6064), reg(0x6077), reg(0x6061), reg(0x60fd)};
    }
    return offsets;
}
}
