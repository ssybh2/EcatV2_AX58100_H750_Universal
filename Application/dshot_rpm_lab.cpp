// Experimental one-channel bidirectional DShot300 receiver for STM32H750.
// THIS CODE MUST NOT BE USED FOR FLIGHT. It is not hardware verified.
// Compile only with -DAIM_DSHOT_RPM_LAB_RX=ON on a propeller-free bench.
// Preserves the default build byte-for-byte outside this opt-in compilation.
#include "dshot_rpm_lab.hpp"

#ifdef AIM_DSHOT_RPM_LAB_RX

#include <cstdint>
#include <atomic>
#include "dshot_rpm_capture_api.hpp"
#include "dshot_rpm_tx.hpp"

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

// Counters increment only on completed TX callback or capture finalization.
// Individual EXTI edges only update the original fixed-size edge array.
struct DiagnosticCounters {
    std::atomic<uint32_t> tx_complete{0};
    std::atomic<uint32_t> capture_calls{0};
    std::atomic<uint32_t> no_start_edge{0};
    std::atomic<uint32_t> edge_overflow{0};
    std::atomic<uint32_t> decode_ok{0};
    std::atomic<uint32_t> decode_fail{0};
    std::atomic<uint32_t> total_edges{0};
    std::atomic<uint32_t> last_edge_count{0};
    std::atomic<uint32_t> last_ok_ms{0};
    std::atomic<uint32_t> no_capture_window{0};
};
DiagnosticCounters diag{};
std::atomic<bool> had_ok{false};
inline void inc(std::atomic<uint32_t> &counter) {
    counter.fetch_add(1, std::memory_order_relaxed);
}

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
    inc(diag.tx_complete);

    // The last DMA completion can precede the end of the final PWM cell.
    // Wait long enough for the DMA tail cells to finish. DShot300 has
    // 3.33 us per bit, so a 5 us turnaround needs oscilloscope verification.
    // This is an EXTI proof of concept, not a production timing guarantee.
    const uint32_t start = DWT->CYCCNT;
    const uint32_t delay_cycles = (SystemCoreClock / 1000000U) * 5U;
    while (static_cast<uint32_t>(DWT->CYCCNT - start) < delay_cycles) {}
    release_all_motor_pins();
}

void finish_capture() {
    // Called at the next EtherCAT command, nominally ~1ms later.
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    const bool capture_armed = armed;
    armed = false;
    inc(diag.capture_calls);
    const uint32_t count = edge_count;
    diag.last_edge_count.store(count, std::memory_order_relaxed);
    diag.total_edges.fetch_add(count, std::memory_order_relaxed);
    if (!capture_armed) {
        inc(diag.no_capture_window);
    } else if (overflow) {
        inc(diag.edge_overflow);
    } else if (!first_falling || count == 0) {
        inc(diag.no_start_edge);
    } else {
        uint32_t snapshot[kCapacity]{};
        for (uint32_t i = 0; i < count; ++i) {
            snapshot[i] = edges[i];
        }
        const uint32_t bitrate = dshot_rpm::reply_bitrate(dshot_rpm::Speed::DSHOT300);
        const uint32_t bit_ticks = (SystemCoreClock + bitrate / 2U) / bitrate;
        const uint32_t now_ms = HAL_GetTick();
        if (task::pwm::dshot_rpm_cache::accept_captured_edges(
                2, 0, snapshot, count, bit_ticks, now_ms, 0xFFFFFFFFUL)) {
            inc(diag.decode_ok);
            diag.last_ok_ms.store(now_ms, std::memory_order_relaxed);
            had_ok.store(true, std::memory_order_release);
        } else {
            inc(diag.decode_fail);
        }
    }
    restore_transmit_pins();
}

void shutdown() {
    restore_transmit_pins();
}

void read_diagnostics(uint32_t out[10], const uint32_t now_ms) {
    if (out == nullptr) { return; }
    out[0] = diag.tx_complete.load(std::memory_order_relaxed);
    out[1] = diag.capture_calls.load(std::memory_order_relaxed);
    out[2] = diag.no_start_edge.load(std::memory_order_relaxed);
    out[3] = diag.edge_overflow.load(std::memory_order_relaxed);
    out[4] = diag.decode_ok.load(std::memory_order_relaxed);
    out[5] = diag.decode_fail.load(std::memory_order_relaxed);
    out[6] = diag.total_edges.load(std::memory_order_relaxed);
    out[7] = diag.last_edge_count.load(std::memory_order_relaxed);
    out[8] = had_ok.load(std::memory_order_acquire)
        ? static_cast<uint32_t>(now_ms - diag.last_ok_ms.load(std::memory_order_relaxed))
        : 0xFFFFFFFFUL;
    out[9] = diag.no_capture_window.load(std::memory_order_relaxed);
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
