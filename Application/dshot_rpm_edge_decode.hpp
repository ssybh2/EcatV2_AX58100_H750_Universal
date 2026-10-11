// Decode edge timestamps of one 750 kbit/s bidirectional DShot reply.
// Portable DSP/protocol layer: does not configure a timer or GPIO.
// A capture driver MUST verify the first edge is falling (idle-high -> low)
// and provide monotonically ordered timer timestamps at a known timer clock.
#ifndef AIM_DSHOT_RPM_EDGE_DECODE_HPP
#define AIM_DSHOT_RPM_EDGE_DECODE_HPP

#include <cstddef>
#include <cstdint>
#include "dshot_rpm_protocol.hpp"

namespace aim::ecat::dshot_rpm {
    constexpr std::size_t REPLY_BITS = 21;
    constexpr std::size_t MAX_REPLY_EDGES = 24;

    // First falling edge starts bit 0 at logic level 0; later edges toggle
    // the level. Timer timestamps use unsigned subtraction, permitting a
    // wrapping TIM2/TIM3 counter provided a reply spans less than 2^31 ticks.
    // ticks_per_bit = capture_timer_hz / 750000 for DShot600 replies.
    inline bool decode_edge_timestamps(const uint32_t *edges,
                                       const std::size_t edge_count,
                                       const uint32_t ticks_per_bit,
                                       uint16_t &payload) {
        if (edges == nullptr || edge_count == 0 ||
            edge_count > MAX_REPLY_EDGES || ticks_per_bit < 8) {
            return false;
        }
        const uint32_t start = edges[0];
        uint32_t raw21 = 0;
        uint32_t level = 0;
        std::size_t next_edge = 1;
        uint32_t previous_offset = 0;

        // Transition jitter should be much smaller than half a bit.
        for (std::size_t index = 1; index < edge_count; ++index) {
            const uint32_t offset = edges[index] - start;
            if (offset <= previous_offset + ticks_per_bit / 2) {
                return false; // unordered timestamps, bouncing or a glitch
            }
            previous_offset = offset;
        }

        for (uint32_t bit = 0; bit < REPLY_BITS; ++bit) {
            const uint32_t center = bit * ticks_per_bit + ticks_per_bit / 2;
            while (next_edge < edge_count &&
                   static_cast<uint32_t>(edges[next_edge] - start) <= center) {
                level ^= 1U;
                ++next_edge;
            }
            raw21 = (raw21 << 1) | level;
        }
        // Ignore an optional line return-to-idle edge at or after the
        // end of the 21-bit symbol. Reject extra edges inside the frame.
        if (next_edge < edge_count) {
            if (edge_count != next_edge + 1 ||
                static_cast<uint32_t>(edges[next_edge] - start) <
                    REPLY_BITS * ticks_per_bit - ticks_per_bit / 4) {
                return false;
            }
        }
        return decode_21bit(raw21, payload);
    }
}

#endif
