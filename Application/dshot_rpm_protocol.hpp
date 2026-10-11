// Bidirectional DShot eRPM telemetry codec (protocol layer only).
// No GPIO / DMA capture is enabled by this header.
// Reference: https://betaflight.com/docs/development/API/Dshot
#ifndef AIM_DSHOT_RPM_PROTOCOL_HPP
#define AIM_DSHOT_RPM_PROTOCOL_HPP

#include <cstdint>

namespace aim::ecat::dshot_rpm {
    constexpr uint16_t INVALID_PAYLOAD = 0xFFFF;
    constexpr uint16_t STOPPED_PAYLOAD = 0x0FFF;

    // Build a 16-bit FC->ESC bidirectional DShot command using the
    // inverted checksum. This function only builds bytes: enabling bDShot
    // also requires inverted electrical levels, idle-high wiring and RX.
    // The separate telemetry-request bit remains zero (single-wire bDShot
    // eRPM feedback is automatic).
    inline uint16_t prepare_bidir_command(const uint16_t throttle) {
        const uint16_t data = static_cast<uint16_t>((throttle & 0x07FFU) << 1);
        const uint16_t crc = static_cast<uint16_t>(~(data ^ (data >> 4) ^ (data >> 8)) & 0x0FU);
        return static_cast<uint16_t>((data << 4) | crc);
    }

    // Invert the 4b/5b GCR table. Unmapped symbols are invalid.
    inline int gcr_nibble(const uint8_t code) {
        switch (code) {
            case 0x19: return 0x0;
            case 0x1B: return 0x1;
            case 0x12: return 0x2;
            case 0x13: return 0x3;
            case 0x1D: return 0x4;
            case 0x15: return 0x5;
            case 0x16: return 0x6;
            case 0x17: return 0x7;
            case 0x1A: return 0x8;
            case 0x09: return 0x9;
            case 0x0A: return 0xA;
            case 0x0B: return 0xB;
            case 0x1E: return 0xC;
            case 0x0D: return 0xD;
            case 0x0E: return 0xE;
            case 0x0F: return 0xF;
            default: return -1;
        }
    }

    // raw21 is a FULL, correctly aligned, 21-bit received waveform,
    // MSB first and including the start bit. Obtaining it requires a
    // bidirectional GPIO / timer / DMA receiver (not implemented yet).
    // Returns false for invalid GCR symbols or inverted-DShot CRC failures.
    inline bool decode_21bit(const uint32_t raw21, uint16_t &payload) {
        if (raw21 > 0x1FFFFF) {
            return false;
        }
        const uint32_t gcr = (raw21 ^ (raw21 >> 1)) & 0xFFFFF;
        uint16_t decoded = 0;
        for (int i = 3; i >= 0; --i) {
            const int nibble = gcr_nibble(static_cast<uint8_t>((gcr >> (i * 5)) & 0x1F));
            if (nibble < 0) {
                return false;
            }
            decoded = static_cast<uint16_t>((decoded << 4) | static_cast<uint16_t>(nibble));
        }
        // ESC reply: CRC nibble is the complement of XOR(data nibbles).
        const uint16_t xor_nibbles = decoded ^ (decoded >> 4) ^ (decoded >> 8) ^ (decoded >> 12);
        if ((xor_nibbles & 0x0F) != 0x0F) {
            return false;
        }
        payload = static_cast<uint16_t>(decoded >> 4);
        // Reject non-stopped zero-period encodings (RPM would be infinite).
        return payload == STOPPED_PAYLOAD || (payload & 0x01FF) != 0;
    }

    // eee mmmmmmmmm -> period (microseconds); 0xFFF means stopped.
    // Does NOT distinguish positive and negative motor rotation.
    inline uint32_t period_us(const uint16_t payload) {
        if (payload == STOPPED_PAYLOAD) {
            return 0;
        }
        if (payload > 0x0FFF || (payload & 0x01FF) == 0) {
            return 0;
        }
        return static_cast<uint32_t>(payload & 0x01FF) << (payload >> 9);
    }

    inline uint32_t electrical_rpm(const uint16_t payload) {
        const uint32_t period = period_us(payload);
        return period == 0 ? 0 : (60000000UL + period / 2) / period;
    }
}

#endif // AIM_DSHOT_RPM_PROTOCOL_HPP
