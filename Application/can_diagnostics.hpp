#ifndef ECATV2_AX58100_H750_UNIVERSAL_CAN_DIAGNOSTICS_HPP
#define ECATV2_AX58100_H750_UNIVERSAL_CAN_DIAGNOSTICS_HPP

#include <cstdint>

extern volatile uint32_t can1_rx_frame_count;
extern volatile uint32_t can2_rx_frame_count;
extern volatile uint32_t can1_rx_fifo_full_count;
extern volatile uint32_t can2_rx_fifo_full_count;
extern volatile uint32_t can1_rx_fifo_lost_count;
extern volatile uint32_t can2_rx_fifo_lost_count;
extern volatile uint32_t can1_rx_read_error_count;
extern volatile uint32_t can2_rx_read_error_count;

#endif // ECATV2_AX58100_H750_UNIVERSAL_CAN_DIAGNOSTICS_HPP
