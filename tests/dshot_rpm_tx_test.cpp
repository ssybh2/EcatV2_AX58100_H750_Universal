// Portable exhaustive bench-only TX waveform test for DShot300 and 600.
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include "../Application/dshot_rpm_tx.hpp"

int main() {
    using namespace aim::ecat::dshot_rpm;
    std::array<uint32_t, DSHOT_TX_DMA_WORDS> waveform{};
    for (const Speed speed : {Speed::DSHOT300, Speed::DSHOT600}) {
        assert(timer_period_ticks(speed) == (speed == Speed::DSHOT300 ? 40U : 20U));
        assert(reply_bitrate(speed) == (speed == Speed::DSHOT300 ? 375000U : 750000U));
        for (uint32_t throttle = 0; throttle < 2048; ++throttle) {
            prepare_bidir_tx_compare(waveform.data(), static_cast<uint16_t>(throttle), speed);
            const uint16_t command = prepare_bidir_command(static_cast<uint16_t>(throttle));
            for (uint32_t bit = 0; bit < 16; ++bit) {
                const bool one = (command & (0x8000U >> bit)) != 0;
                assert(waveform[bit] == (one ? one_low_ticks(speed) : zero_low_ticks(speed)));
                assert(waveform[bit] < timer_period_ticks(speed));
            }
            assert(waveform[16] == 0 && waveform[17] == 0);
        }
    }
    prepare_bidir_tx_compare(waveform.data(), 1046U, Speed::DSHOT300);
    assert(prepare_bidir_command(1046U) == 0x82C9U);
    std::cout << "PASS: all 2048 throttle words at both DShot300/600; inverted CRC, PWM2 timing\n";
}
