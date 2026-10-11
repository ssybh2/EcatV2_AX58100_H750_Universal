// Hook for a future TIM/DMA bidirectional DShot receiver.
// The receiver must supply a correctly timed and aligned 21-bit GCR frame.
// Until this is called, the RPM PDO stays INVALID on every channel.
#ifndef AIM_DSHOT_RPM_CAPTURE_API_HPP
#define AIM_DSHOT_RPM_CAPTURE_API_HPP

#include <cstdint>

namespace aim::ecat::task::pwm::dshot_rpm_cache {
    // dshot_id=1/2; channel=0..3; timestamp_ms=HAL_GetTick().
    // Returns true only for a valid GCR+CRC eRPM frame.
    bool accept_captured_frame(uint8_t dshot_id, uint8_t channel,
                               uint32_t raw21, uint32_t timestamp_ms);
}

#endif
