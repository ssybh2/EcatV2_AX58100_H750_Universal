//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"
#include "hipnuc_imu_diag.hpp"

extern "C" {
#include "fdcan.h"
}

namespace aim::ecat::task::hipnuc_imu {
    namespace {
        constexpr size_t MAX_HIPNUC_IMUS = 8;

        struct ImuAssemblyState {
            const HIPNUC_IMU_CAN *owner{};
            uint8_t pending_buf[21]{};
            uint8_t stage{};
            uint32_t complete_samples{};
            uint32_t incomplete_samples{};
            uint32_t last_complete_tick{};
        };

        ImuAssemblyState imu_states[MAX_HIPNUC_IMUS]{};

        ImuAssemblyState *get_assembly_state(const HIPNUC_IMU_CAN *owner) {
            for (auto &state: imu_states) {
                if (state.owner == owner) {
                    return &state;
                }
            }

            for (auto &state: imu_states) {
                if (state.owner == nullptr) {
                    state.owner = owner;
                    return &state;
                }
            }

            return nullptr;
        }
    }

    void get_diag_snapshot(HipnucImuDiagSnapshot *snapshot) {
        if (snapshot == nullptr) {
            return;
        }

        for (uint8_t i = 0; i < HIPNUC_DIAG_IMU_COUNT; ++i) {
            snapshot->sample_seq[i] = static_cast<uint16_t>(imu_states[i].complete_samples & 0xFFFFU);
            snapshot->incomplete_samples[i] = static_cast<uint16_t>(imu_states[i].incomplete_samples & 0xFFFFU);
        }
    }

    HIPNUC_IMU_CAN::HIPNUC_IMU_CAN(buffer::Buffer *buffer) : CanRunnable(false, TaskType::HIPNUC_IMU_CAN) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN_1M);
        can_id_type_ = FDCAN_STANDARD_ID;

        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 0x01: {
                can_inst_ = &hfdcan1;
                break;
            }
            case 0x02: {
                can_inst_ = &hfdcan2;
                break;
            }
            default: {
                can_inst_ = nullptr;
                break;
            }
        }

        packet1_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
        packet2_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
        packet3_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);

        /* Reserve one fixed assembly slot for this IMU task. */
        get_assembly_state(this);
    }

    void HIPNUC_IMU_CAN::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        if (can_inst_ == nullptr || rx_header == nullptr || rx_data == nullptr) {
            return;
        }

        if (rx_header->Identifier != packet1_id_
            && rx_header->Identifier != packet2_id_
            && rx_header->Identifier != packet3_id_) {
            return;
        }

        ImuAssemblyState *state = get_assembly_state(this);
        if (state == nullptr) {
            return;
        }

        if (packet1_id_ == rx_header->Identifier) {
            /* A new packet1 starts a new sample. If the previous sample was
             * incomplete, count it and discard the partial data. */
            if (state->stage != 0U) {
                state->incomplete_samples++;
            }
            memcpy(state->pending_buf, rx_data, 8);
            state->stage = 1U;
            return;
        }

        if (packet2_id_ == rx_header->Identifier) {
            if (state->stage != 1U) {
                state->incomplete_samples++;
                state->stage = 0U;
                return;
            }
            memcpy(state->pending_buf + 8, rx_data, 8);
            state->stage = 2U;
            return;
        }

        /* packet3: only commit after packet1 -> packet2 -> packet3 arrived. */
        if (state->stage != 2U) {
            state->incomplete_samples++;
            state->stage = 0U;
            return;
        }

        memcpy(state->pending_buf + 16, rx_data, 5);
        buf_.write(state->pending_buf, 21);
        state->complete_samples++;
        state->last_complete_tick = HAL_GetTick();
        state->stage = 0U;
    }

    void HIPNUC_IMU_CAN::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t current_buf[21] = {};
        buf_.read(current_buf, 21);
        slave_to_master_buf->write(current_buf, 21);
    }
}
