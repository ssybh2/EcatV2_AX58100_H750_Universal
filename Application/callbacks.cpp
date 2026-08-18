//
// Created by Hang XU on 21/10/2025.
//
#include "peripheral_utils.hpp"
#include "task_manager.hpp"
#include "c_task_warpper.h"
#include "can_diagnostics.hpp"

#include <atomic>

using namespace aim::ecat::task;
using namespace aim::hardware;

namespace {
    std::atomic<uint32_t> can1_rx_frame_count{0};
    std::atomic<uint32_t> can2_rx_frame_count{0};
    std::atomic<uint32_t> can1_rx_fifo_full_count{0};
    std::atomic<uint32_t> can2_rx_fifo_full_count{0};
    std::atomic<uint32_t> can1_rx_fifo_lost_count{0};
    std::atomic<uint32_t> can2_rx_fifo_lost_count{0};
    std::atomic<uint32_t> can1_rx_read_error_count{0};
    std::atomic<uint32_t> can2_rx_read_error_count{0};
}

namespace aim::ecat::diagnostics {
    void get_can_rx_diagnostics(CanRxDiagnostics *snapshot) {
        if (snapshot == nullptr) {
            return;
        }

        snapshot->can1_rx_frame_count = can1_rx_frame_count.load(std::memory_order_relaxed);
        snapshot->can2_rx_frame_count = can2_rx_frame_count.load(std::memory_order_relaxed);
        snapshot->can1_rx_fifo_full_count = can1_rx_fifo_full_count.load(std::memory_order_relaxed);
        snapshot->can2_rx_fifo_full_count = can2_rx_fifo_full_count.load(std::memory_order_relaxed);
        snapshot->can1_rx_fifo_lost_count = can1_rx_fifo_lost_count.load(std::memory_order_relaxed);
        snapshot->can2_rx_fifo_lost_count = can2_rx_fifo_lost_count.load(std::memory_order_relaxed);
        snapshot->can1_rx_read_error_count = can1_rx_read_error_count.load(std::memory_order_relaxed);
        snapshot->can2_rx_read_error_count = can2_rx_read_error_count.load(std::memory_order_relaxed);
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_USART1))->is_busy_.clear(); // NOLINT
    } else if (huart->Instance == UART4) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART4))->is_busy_.clear(); // NOLINT
    } else if (huart->Instance == UART8) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART8))->is_busy_.clear(); // NOLINT
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_uart_task.get()) {
            continue;
        }
        if (huart->Instance != conf->runnable->get_peripheral<peripheral::UartPeripheral>()->huart_->Instance) {
            continue;
        }

        static_cast<UartRunnable *>(conf->runnable.get())->uart_dma_tx_finished_callback(); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
// ReSharper disable once CppParameterMayBeConst
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_uart_task.get()) {
            continue;
        }
        if (huart->Instance != conf->runnable->get_peripheral<peripheral::UartPeripheral>()->huart_->Instance) {
            continue;
        }

        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->recv_buf_->reset_index();
        static_cast<UartRunnable *>(conf->runnable.get())->uart_recv(Size); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_USART1))->is_busy_.clear(); // NOLINT
    } else if (huart->Instance == UART4) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART4))->is_busy_.clear(); // NOLINT
    } else if (huart->Instance == UART8) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART8))->is_busy_.clear(); // NOLINT
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_uart_task.get()) {
            continue;
        }
        if (huart->Instance != conf->runnable->get_peripheral<peripheral::UartPeripheral>()->huart_->Instance) {
            continue;
        }

        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->recv_buf_->reset();
        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->send_buf_->reset();
        static_cast<UartRunnable *>(conf->runnable.get())->uart_err(); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C3) {
        static_cast<peripheral::I2CPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_I2C3))->is_busy_.clear(); // NOLINT
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_i2c_task.get()) {
            continue;
        }
        if (hi2c->Instance != conf->runnable->get_peripheral<peripheral::I2CPeripheral>()->hi2c_->Instance) {
            continue;
        }

        static_cast<I2CRunnable *>(conf->runnable.get())->i2c_dma_tx_finished_callback(); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_i2c_task.get()) {
            continue;
        }
        if (hi2c->Instance != conf->runnable->get_peripheral<peripheral::I2CPeripheral>()->hi2c_->Instance) {
            continue;
        }

        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->recv_buf_->reset_index();
        static_cast<I2CRunnable *>(conf->runnable.get())->i2c_recv(); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C3) {
        static_cast<peripheral::I2CPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_I2C3))->is_busy_.clear(); // NOLINT
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_i2c_task.get()) {
            continue;
        }
        if (hi2c->Instance != conf->runnable->get_peripheral<peripheral::I2CPeripheral>()->hi2c_->Instance) {
            continue;
        }

        conf->runnable->get_peripheral<peripheral::I2CPeripheral>()->recv_buf_->reset();
        conf->runnable->get_peripheral<peripheral::I2CPeripheral>()->send_buf_->reset();
        static_cast<I2CRunnable *>(conf->runnable.get())->i2c_err(); // NOLINT
    }
}

void process_can_data(const FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8]) {
    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_can_task.get()) {
            continue;
        }

        auto *can_task = static_cast<CanRunnable *>(conf->runnable.get()); // NOLINT
        if (can_task->can_inst_ == nullptr) {
            continue;
        }
        if (hfdcan->Instance != can_task->can_inst_->Instance) {
            continue;
        }
        if (rx_header->IdType != can_task->can_id_type_) {
            continue;
        }

        can_task->can_recv(rx_header, rx_data);
    }
}

/*
 * Drain the complete FIFO on every interrupt instead of consuming one frame.
 * With three 500 Hz IMUs on one bus, several frames can arrive back-to-back.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) {
        can1_rx_fifo_full_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        can1_rx_fifo_lost_count.fetch_add(1, std::memory_order_relaxed);
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            can1_rx_read_error_count.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        can1_rx_frame_count.fetch_add(1, std::memory_order_relaxed);
        process_can_data(hfdcan, &rx_header, rx_data);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL) {
        can2_rx_fifo_full_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) {
        can2_rx_fifo_lost_count.fetch_add(1, std::memory_order_relaxed);
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) {
            can2_rx_read_error_count.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        can2_rx_frame_count.fetch_add(1, std::memory_order_relaxed);
        process_can_data(hfdcan, &rx_header, rx_data);
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
    if (const TIM_HandleTypeDef *htim = static_cast<TIM_HandleTypeDef *>(hdma->Parent);
        hdma == htim->hdma[TIM_DMA_ID_CC1]) {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
    } else if (hdma == htim->hdma[TIM_DMA_ID_CC2]) {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
    } else if (hdma == htim->hdma[TIM_DMA_ID_CC3]) {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
    } else if (hdma == htim->hdma[TIM_DMA_ID_CC4]) {
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (conf->runnable->task_type == TaskType::DSHOT) {
            static_cast<pwm::DSHOT600 *>(conf->runnable.get())->on_packet_sent(); // NOLINT
        }
    }
}
