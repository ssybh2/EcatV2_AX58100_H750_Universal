// Experimental one-channel bidirectional DShot600 receiver for STM32H750.
// THIS CODE MUST NOT BE USED FOR FLIGHT. It is not hardware verified.
// Compile only with -DAIM_DSHOT_RPM_LAB_RX=ON on a propeller-free bench.
// Preserves the default build byte-for-byte outside this opt-in compilation.
#include "dshot_rpm_lab.hpp"

#ifdef AIM_DSHOT_RPM_LAB_RX

#include <cstdint>
#include "dshot_rpm_capture_api.hpp"

extern "C" {
#include "tim.h"
}

namespace aim::ecat::dshot_rpm::lab {
namespace {
constexpr uint16_t kMotorPins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
constexpr uint8_t kAllDmaChannels = 0x0F;
constexpr uint8_t kCapacity = 24;

// IRQ -> process-data handoff uses volatile rather than DMA for this
// SINGLE-CHANNEL feasibility experiment. No RTOS calls in the edge ISR.
volatile bool armed = false;
volatile bool first_falling = false;
volatile bool overflow = false;
volatile uint8_t edge_count = 0;
volatile uint8_t dma_finished = 0;
volatile uint32_t edges[kCapacity] = {};

void enable_cycle_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void restore_transmit_pins() {
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    armed = false;
    GPIO_InitTypeDef gpio{};
    gpio.Pin = kMotorPins;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void release_all_motor_pins() {
    // TIM PWM2 has idle high, but it is push-pull; all 4 pins must be
    // released BEFORE any ESC reply to prevent output-driver contention.
    GPIO_InitTypeDef gpio{};
    gpio.Pin = kMotorPins;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    edge_count = 0;
    first_falling = false;
    overflow = false;
    armed = true;
    HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void on_edge(const uint32_t cycle, const bool high) {
    if (!armed) {
        return;
    }
    if (!first_falling) {
        if (high) {
            return; // wait for the start-bit falling edge
        }
        first_falling = true;
    }
    if (edge_count < kCapacity) {
        edges[edge_count++] = cycle;
    } else {
        overflow = true;
    }
}
} // namespace

void configure_pwm2(TIM_HandleTypeDef *htim) {
    // Must run before HAL_TIM_PWM_Start. PWM2 is the inverted on-wire
    // form needed by bDShot. All original PWM1 timings remain untouched
    // in the normal build.
    enable_cycle_counter();
    TIM_OC_InitTypeDef cfg{};
    cfg.OCMode = TIM_OCMODE_PWM2;
    cfg.Pulse = 0;
    cfg.OCPolarity = TIM_OCPOLARITY_HIGH;
    cfg.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &cfg, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(htim, &cfg, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(htim, &cfg, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(htim, &cfg, TIM_CHANNEL_4);
}

void begin_transmission() {
    dma_finished = 0;
}

void on_tx_dma_finished(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma) {
    if (htim != &htim2 || armed) {
        return;
    }
    for (uint8_t ch = 0; ch < 4; ++ch) {
        if (hdma == htim->hdma[TIM_DMA_ID_CC1 + ch]) {
            dma_finished = static_cast<uint8_t>(dma_finished | (1U << ch));
            break;
        }
    }
    if (dma_finished != kAllDmaChannels) {
        return;
    }
    dma_finished = 0;

    // The last DMA completion can precede the end of the final PWM cell.
    // Give the timer >2 complete bit cells (DShot600 bit ~1.67us) before
    // releasing the pins, still well ahead of the ESC ~30us response.
    const uint32_t start = DWT->CYCCNT;
    const uint32_t delay_cycles = (SystemCoreClock / 1000000U) * 5U;
    while (static_cast<uint32_t>(DWT->CYCCNT - start) < delay_cycles) {}
    release_all_motor_pins();
}

void finish_capture() {
    // Called at the next EtherCAT command, nominally ~1ms later.
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    armed = false;
    if (first_falling && !overflow && edge_count != 0) {
        uint32_t snapshot[kCapacity]{};
        const uint8_t count = edge_count;
        for (uint8_t i = 0; i < count; ++i) {
            snapshot[i] = edges[i];
        }
        const uint32_t bit_ticks = (SystemCoreClock + 375000U) / 750000U;
        task::pwm::dshot_rpm_cache::accept_captured_edges(
            2, 0, snapshot, count, bit_ticks, HAL_GetTick(), 0xFFFFFFFFUL);
    }
    restore_transmit_pins();
}

void shutdown() {
    restore_transmit_pins();
}

void record_exti_edge(const uint32_t cycle, const bool high) {
    on_edge(cycle, high);
}
} // namespace aim::ecat::dshot_rpm::lab

extern "C" void EXTI0_IRQHandler(void) {
    // Record time before expensive HAL callback dispatch.
    const uint32_t cycle = DWT->CYCCNT;
    const bool high = (GPIOA->IDR & GPIO_PIN_0) != 0;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    // Handler must not touch any RTOS API.
    // Calls into the lab-owned edge accumulator below.
    aim::ecat::dshot_rpm::lab::record_exti_edge(cycle, high);
}
#endif
