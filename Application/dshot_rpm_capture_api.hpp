// Hook for a future TIM/DMA bidirectional DShot receiver.
// The receiver must supply a correctly timed and aligned 21-bit GCR frame.
// Until this is called, the RPM PDO stays INVALID on every channel.
#ifndef AIM_DSHOT_RPM_CAPTURE_API_HPP
#define AIM_DSHOT_RPM_CAPTURE_API_HPP

#include <cstdint>
#include <cstddef>

namespace aim::ecat::task::pwm::dshot_rpm_cache {
    // dshot_id=1/2; channel=0..3; timestamp_ms=HAL_GetTick().
    // Returns true only for a valid GCR+CRC eRPM frame.
    bool accept_captured_frame(uint8_t dshot_id, uint8_t channel,
                               uint32_t raw21, uint32_t timestamp_ms);

    // Process timer input-capture edge timestamps (not raw sampled bits).
    // First edge MUST be verified falling by the capture driver.
    // For 12 MHz capture clock use ticks_per_bit=16 (DShot600 RX 750kbit/s).
    // No hardware capture driver is enabled by this API alone.
    bool accept_captured_edges(uint8_t dshot_id, uint8_t channel,
                               const uint32_t *edges, std::size_t edge_count,
                               uint32_t ticks_per_bit, uint32_t timestamp_ms);
}

#endif
