#include <array>
#include <cstdlib>
#include <iostream>

#include <uvgv3crtp/v3c_api.h>

int main() {
    uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES] = {};
    std::fill(std::begin(ports), std::end(ports), static_cast<uint16_t>(8890));

    std::cout << "Creating V3C sender...\n" << std::flush;
    uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender> state(
        uvgV3CRTP::INIT_FLAGS::VPS | uvgV3CRTP::INIT_FLAGS::AD | uvgV3CRTP::INIT_FLAGS::OVD |
            uvgV3CRTP::INIT_FLAGS::GVD | uvgV3CRTP::INIT_FLAGS::AVD,
        "127.0.0.1",
        ports);

    if (state.get_error_flag() != uvgV3CRTP::ERROR_TYPE::OK) {
        std::cerr << "V3C sender error: " << state.get_error_msg() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "V3C sender OK\n" << std::flush;
    return EXIT_SUCCESS;
}
