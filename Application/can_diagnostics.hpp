#ifndef ECATV2_AX58100_H750_UNIVERSAL_CAN_DIAGNOSTICS_HPP
#define ECATV2_AX58100_H750_UNIVERSAL_CAN_DIAGNOSTICS_HPP

#include <cstdint>

namespace aim::ecat::diagnostics {
    struct CanRxDiagnostics {
        uint32_t can1_rx_frame_count{};
        uint32_t can2_rx_frame_count{};
        uint32_t can1_rx_fifo_full_count{};
        uint32_t can2_rx_fifo_full_count{};
        uint32_t can1_rx_fifo_lost_count{};
        uint32_t can2_rx_fifo_lost_count{};
        uint32_t can1_rx_read_error_count{};
        uint32_t can2_rx_read_error_count{};
    };

    void get_can_rx_diagnostics(CanRxDiagnostics *snapshot);
}

#endif // ECATV2_AX58100_H750_UNIVERSAL_CAN_DIAGNOSTICS_HPP
