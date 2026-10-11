// Bidirectional DShot600 TX compare values (protocol only).
// These CCR values MUST be output with TIM_OCMODE_PWM2 and an idle-high pin.
// Never use with the original PWM1 transmitter: it would not be bDShot.
#ifndef AIM_DSHOT_RPM_TX_HPP
#define AIM_DSHOT_RPM_TX_HPP

#include <cstddef>
#include <cstdint>
#include "dshot_rpm_protocol.hpp"

namespace aim::ecat::dshot_rpm {
    constexpr std::size_t DSHOT_TX_DMA_WORDS = 18;
    constexpr uint32_t DSHOT_0_COMPARE = 7;
    constexpr uint32_t DSHOT_1_COMPARE = 14;

    inline void prepare_bidir_tx_compare(uint32_t *output, const uint16_t throttle) {
        if (output == nullptr) {
            return;
        }
        const uint16_t limited = throttle > 2047U ? 2047U : throttle;
        uint16_t packet = prepare_bidir_command(limited);
        for (std::size_t i = 0; i < 16; ++i) {
            // CCR is the LOW time in PWM2, not the high time of PWM1.
            output[i] = (packet & 0x8000U) ? DSHOT_1_COMPARE : DSHOT_0_COMPARE;
            packet = static_cast<uint16_t>(packet << 1);
        }
        // Two idle-high cells at the end of the DMA command.
        output[16] = 0;
        output[17] = 0;
    }
}
#endif
