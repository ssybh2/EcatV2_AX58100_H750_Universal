// Optional read-only EtherCAT task for future bidirectional DShot capture.
// Important: no RX DMA source feeds this cache yet; all channels report invalid.
// The legacy DSHOT task continues transmitting exactly as before.
#include <atomic>
#include <cstdint>

#include "buffer_utils.hpp"
#include "dshot_rpm_protocol.hpp"
#include "dshot_rpm_capture_api.hpp"
#include "task_defs.hpp"

extern "C" {
#include "stm32h7xx_hal.h"
}

namespace aim::ecat::task::pwm {
    namespace dshot_rpm_cache {
        struct Channel {
            std::atomic<uint16_t> payload{dshot_rpm::INVALID_PAYLOAD};
            std::atomic<uint32_t> timestamp_ms{0};
            std::atomic<bool> received{false};
        };

        static Channel samples[2][4];
        constexpr uint32_t MAX_SAMPLE_AGE_MS = 100;

        // Future RX-DMA completion hook. A failed frame leaves the last
        // valid sample intact until MAX_SAMPLE_AGE_MS elapses.
        bool accept_captured_frame(const uint8_t dshot_id, const uint8_t channel,
                                   const uint32_t raw21, const uint32_t timestamp_ms) {
            if (dshot_id < 1 || dshot_id > 2 || channel >= 4) {
                return false;
            }
            uint16_t payload = 0;
            if (!dshot_rpm::decode_21bit(raw21, payload)) {
                return false;
            }
            Channel &slot = samples[dshot_id - 1][channel];
            slot.payload.store(payload, std::memory_order_relaxed);
            slot.timestamp_ms.store(timestamp_ms, std::memory_order_relaxed);
            slot.received.store(true, std::memory_order_release);
            return true;
        }

        bool read(const uint8_t dshot_id, const uint8_t channel, const uint32_t now_ms, uint16_t &payload) {
            if (dshot_id < 1 || dshot_id > 2 || channel >= 4) {
                return false;
            }
            Channel &slot = samples[dshot_id - 1][channel];
            if (!slot.received.load(std::memory_order_acquire)) {
                return false;
            }
            const uint32_t timestamp = slot.timestamp_ms.load(std::memory_order_relaxed);
            if (static_cast<uint32_t>(now_ms - timestamp) > MAX_SAMPLE_AGE_MS) {
                return false;
            }
            payload = slot.payload.load(std::memory_order_relaxed);
            return true;
        }
    }

    DSHOT_RPM::DSHOT_RPM(buffer::Buffer *buffer) : CustomRunnable(false, TaskType::DSHOT_RPM) {
        dshot_id_ = buffer->read_uint8(buffer::EndianType::LITTLE);
        if (dshot_id_ != 1 && dshot_id_ != 2) {
            dshot_id_ = 0; // safely publish all-invalid for bad configuration
        }
    }

    void DSHOT_RPM::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint16_t payloads[4] = {
            dshot_rpm::INVALID_PAYLOAD, dshot_rpm::INVALID_PAYLOAD,
            dshot_rpm::INVALID_PAYLOAD, dshot_rpm::INVALID_PAYLOAD
        };
        uint8_t valid_mask = 0;
        const uint32_t now_ms = HAL_GetTick();
        for (uint8_t channel = 0; channel < 4; ++channel) {
            if (dshot_rpm_cache::read(dshot_id_, channel, now_ms, payloads[channel])) {
                valid_mask = static_cast<uint8_t>(valid_mask | (1U << channel));
            }
            slave_to_master_buf->write_uint16(buffer::EndianType::LITTLE, payloads[channel]);
        }
        slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, valid_mask);
    }
}
