// OPT-IN bench prototype, not a production-ready ESC interface.
// TIM2 / software dshot_id=2, PA0 is the only decoded return channel.
// Other three motor pins are released during ESC response to avoid contention.
// Do not enable until signal levels and timing have been checked on a scope.
#ifndef AIM_DSHOT_RPM_LAB_HPP
#define AIM_DSHOT_RPM_LAB_HPP

#include "stm32h7xx_hal.h"
#include <cstdint>

namespace aim::ecat::dshot_rpm::lab {
#ifdef AIM_DSHOT_RPM_LAB_RX
    void configure_pwm2(TIM_HandleTypeDef *htim);
    void begin_transmission();
    void on_tx_dma_finished(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
    void finish_capture();
    void shutdown();
    // Ten uint32 diagnostic values; see docs/DSHOT300_RX_DIAGNOSTICS.md.
    void read_diagnostics(uint32_t out[10], uint32_t now_ms);
#endif
}

#endif
