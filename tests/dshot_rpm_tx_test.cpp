// Host-side test for DShot600 inverted-CRC TX DMA encoding.
// NOTE: actual H750 PWM2 output, tri-state and RX capture are not validated here.
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include "../Application/dshot_rpm_tx.hpp"

int main() {
    using namespace aim::ecat::dshot_rpm;
    std::array<uint32_t, DSHOT_TX_DMA_WORDS> waveform{};
    for (uint32_t throttle = 0; throttle < 2048; ++throttle) {
        prepare_bidir_tx_compare(waveform.data(), static_cast<uint16_t>(throttle));
        const uint16_t command = prepare_bidir_command(static_cast<uint16_t>(throttle));
        for (uint32_t bit = 0; bit < 16; ++bit) {
            const bool one = (command & (0x8000U >> bit)) != 0;
            assert(waveform[bit] == (one ? DSHOT_1_COMPARE : DSHOT_0_COMPARE));
        }
        assert(waveform[16] == 0 && waveform[17] == 0);
    }
    prepare_bidir_tx_compare(waveform.data(), 1046U);
    assert(prepare_bidir_command(1046U) == 0x82C9U); // Betaflight vector
    std::cout << "PASS: 2048 inverted-CRC DShot600 TX DMA command frames\n";
}
