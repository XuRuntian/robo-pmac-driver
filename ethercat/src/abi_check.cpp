#include <ecrt.h>
#include <iostream>

int main() {
    // This is the only executable linked to the REAL library. No /dev access.
    const auto version = ecrt_version_magic();
    if (version != ECRT_VERSION_MAGIC) {
        std::cerr << "IgH header/library API mismatch\n";
        return 1;
    }
    std::cout << "Real libethercat API " << (version >> 8) << '.' << (version & 0xff)
              << ": header/library match; no master requested\n";
}
