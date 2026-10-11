// Portable inverted-CRC bidirectional DShot TX DMA waveform.
// Timer base: 12 MHz (83.33 ns per tick); DShot300=40 ticks/bit,
// DShot600=20 ticks/bit. PWM2 mode is required for inverted levels.
// Original type-4 DShot600 PWM1 encoder is independent of this file.
#ifndef AIM_DSHOT_RPM_TX_HPP
#define AIM_DSHOT_RPM_TX_HPP

#include <cstddef>
#include <cstdint>
#include "dshot_rpm_protocol.hpp"

namespace aim::ecat::dshot_rpm {
    enum class Speed : uint8_t { DSHOT300 = 0, DSHOT600 = 1 };
    constexpr std::size_t DSHOT_TX_DMA_WORDS = 18;

    constexpr uint32_t timer_period_ticks(const Speed speed) {
        return speed == Speed::DSHOT300 ? 40U : 20U;
    }
    constexpr uint32_t zero_low_ticks(const Speed speed) {
        return speed == Speed::DSHOT300 ? 14U : 7U;
    }
    constexpr uint32_t one_low_ticks(const Speed speed) {
        return speed == Speed::DSHOT300 ? 28U : 14U;
    }
    constexpr uint32_t reply_bitrate(const Speed speed) {
        // GCR 5/4 symbols are sent at 1.25x DShot command bit rate.
        return speed == Speed::DSHOT300 ? 375000U : 750000U;
    }

    inline void prepare_bidir_tx_compare(uint32_t *output, const uint16_t throttle,
                                         const Speed speed = Speed::DSHOT600) {
        if (output == nullptr) {
            return;
        }
        const uint16_t limited = throttle > 2047U ? 2047U : throttle;
        uint16_t packet = prepare_bidir_command(limited);
        for (std::size_t i = 0; i < 16; ++i) {
            // PWM2 active-high: CCR is the initial LOW duration.
            output[i] = (packet & 0x8000U) ? one_low_ticks(speed) : zero_low_ticks(speed);
            packet = static_cast<uint16_t>(packet << 1);
        }
        output[16] = 0;
        output[17] = 0;
    }
}
#endif
