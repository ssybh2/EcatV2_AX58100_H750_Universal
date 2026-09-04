//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"
#include "hipnuc_imu_diag.hpp"

#include <atomic>

extern "C" {
#include "fdcan.h"
}

namespace aim::ecat::task::hipnuc_imu {
    namespace {
        constexpr size_t MAX_HIPNUC_IMUS = 8;
        constexpr size_t HIPNUC_PAYLOAD_SIZE = 21;
        constexpr size_t HIPNUC_COMMITTED_SNAPSHOT_SIZE = 23;

        struct ImuAssemblyState {
            const HIPNUC_IMU_CAN *owner{};
            uint8_t pending_buf[21]{};
            uint8_t stage{};
            ThreadSafeCounter complete_samples{};
            ThreadSafeCounter incomplete_samples{};
            ThreadSafeTimestamp last_complete_tick{};

            /*
             * The CAN IRQ commits payload + sequence as one immutable snapshot.
             * EtherCAT reads only the published slot, so a packet3 IRQ cannot
             * pair sample N payload with sample N+1 sequence diagnostics.
             *
             * snapshot_generation lets the reader detect the rare case where a
             * second publication happens while its 23-byte memcpy is in flight.
             */
            uint8_t committed_snapshot[2][23]{};
            std::atomic<uint8_t> active_snapshot{};
            std::atomic<uint32_t> snapshot_generation{};
            ThreadSafeValue<uint16_t> last_pdo_sample_seq{};
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

        void read_committed_snapshot(const ImuAssemblyState *state, uint8_t current_snapshot[23]) {
            if (state == nullptr || current_snapshot == nullptr) {
                return;
            }

            while (true) {
                const uint32_t generation_before = state->snapshot_generation.load(std::memory_order_acquire);
                const uint8_t read_snapshot_idx = state->active_snapshot.load(std::memory_order_acquire) & 0x01U;

                memcpy(current_snapshot,
                       state->committed_snapshot[read_snapshot_idx],
                       HIPNUC_COMMITTED_SNAPSHOT_SIZE);

                const uint32_t generation_after = state->snapshot_generation.load(std::memory_order_acquire);
                if (generation_before == generation_after) {
                    return;
                }
            }
        }
    }

    void get_diag_snapshot(HipnucImuDiagSnapshot *snapshot) {
        if (snapshot == nullptr) {
            return;
        }

        for (uint8_t i = 0; i < HIPNUC_DIAG_IMU_COUNT; ++i) {
            /* This is the sequence captured with the payload already copied
             * into the current EtherCAT PDO, not the live CAN completion count. */
            snapshot->sample_seq[i] = imu_states[i].last_pdo_sample_seq.get();
            snapshot->incomplete_samples[i] = static_cast<uint16_t>(imu_states[i].incomplete_samples.get() & 0xFFFFU);
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
                state->incomplete_samples.increment();
            }
            memcpy(state->pending_buf, rx_data, 8);
            state->stage = 1U;
            return;
        }

        if (packet2_id_ == rx_header->Identifier) {
            if (state->stage != 1U) {
                state->incomplete_samples.increment();
                state->stage = 0U;
                return;
            }
            memcpy(state->pending_buf + 8, rx_data, 8);
            state->stage = 2U;
            return;
        }

        /* packet3: only commit after packet1 -> packet2 -> packet3 arrived. */
        if (state->stage != 2U) {
            state->incomplete_samples.increment();
            state->stage = 0U;
            return;
        }

        memcpy(state->pending_buf + 16, rx_data, 5);

        const uint16_t next_sample_seq = static_cast<uint16_t>((state->complete_samples.get() + 1U) & 0xFFFFU);
        const uint8_t write_snapshot_idx =
                (state->active_snapshot.load(std::memory_order_acquire) & 0x01U) == 0U ? 1U : 0U;

        memcpy(state->committed_snapshot[write_snapshot_idx],
               state->pending_buf,
               HIPNUC_PAYLOAD_SIZE);
        state->committed_snapshot[write_snapshot_idx][21] = static_cast<uint8_t>(next_sample_seq & 0xFFU);
        state->committed_snapshot[write_snapshot_idx][22] = static_cast<uint8_t>((next_sample_seq >> 8U) & 0xFFU);

        /* Publish only after all 23 bytes are complete. */
        state->active_snapshot.store(write_snapshot_idx, std::memory_order_release);
        state->snapshot_generation.fetch_add(1U, std::memory_order_release);

        state->complete_samples.increment();
        state->last_complete_tick.set_current();
        state->stage = 0U;
    }

    void HIPNUC_IMU_CAN::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t current_snapshot[23] = {};
        ImuAssemblyState *state = get_assembly_state(this);

        if (state != nullptr) {
            read_committed_snapshot(state, current_snapshot);
        }

        slave_to_master_buf->write(current_snapshot, HIPNUC_PAYLOAD_SIZE);

        if (state != nullptr) {
            const uint16_t snapshot_seq = static_cast<uint16_t>(current_snapshot[21]) |
                                          static_cast<uint16_t>(current_snapshot[22]) << 8U;
            state->last_pdo_sample_seq.set(snapshot_seq);
        }
    }
}
