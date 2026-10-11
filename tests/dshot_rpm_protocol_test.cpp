// Host-only independent protocol round-trip tests. No STM32 HAL required.
#include <cassert>
#include <cstdint>
#include <iostream>

#include "../Application/dshot_rpm_protocol.hpp"

using namespace aim::ecat::dshot_rpm;

static uint32_t encode_independent(const uint16_t payload, const int force_crc = -1) {
    static constexpr uint8_t gcr_table[16] = {
        0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
        0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
    };
    assert(payload <= 0x0FFF);
    const uint8_t crc = force_crc >= 0 ? static_cast<uint8_t>(force_crc) :
        static_cast<uint8_t>((~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F);
    const uint16_t frame = static_cast<uint16_t>((payload << 4) | crc);
    uint32_t gcr20 = 0;
    for (int i = 3; i >= 0; --i) {
        gcr20 = (gcr20 << 5) | gcr_table[(frame >> (i * 4)) & 0x0F];
    }
    // Initial waveform level 0, followed by 20 GCR-controlled transitions.
    uint32_t raw21 = 0;
    uint32_t level = 0;
    for (int bit = 19; bit >= 0; --bit) {
        level ^= (gcr20 >> bit) & 1U;
        raw21 = (raw21 << 1) | level;
    }
    return raw21; // leading 21st bit is zero by construction
}

int main() {
    for (uint16_t data : {uint16_t(0xFFF), uint16_t(0x001), uint16_t(0x1FF),
                          uint16_t(0x810), uint16_t(0x123), uint16_t(0xF0A)}) {
        uint16_t decoded = 0;
        const uint32_t raw = encode_independent(data);
        assert(decode_21bit(raw, decoded));
        assert(decoded == data);
        assert(!decode_21bit(raw | (1UL << 21), decoded));
    }

    // Exhaustive valid eRPM payloads; invalid zero mantissa is rejected.
    for (uint16_t p = 0; p <= 0x0FFF; ++p) {
        uint16_t decoded = 0;
        const bool valid = decode_21bit(encode_independent(p), decoded);
        const bool should_be_valid = p == 0x0FFF || (p & 0x01FF) != 0;
        assert(valid == should_be_valid);
        if (valid) assert(decoded == p);
    }

    uint16_t unused = 0;
    // A checksum mismatch must fail even when all GCR codes remain legal.
    assert(!decode_21bit(encode_independent(0x123, 0), unused));
    // Impossible GCR code (all zero) must fail.
    assert(!decode_21bit(0, unused));
    assert(period_us(0xFFF) == 0);
    assert(electrical_rpm(0xFFF) == 0);
    assert(period_us(0x001) == 1);
    assert(electrical_rpm(0x001) == 60000000);
    assert(period_us(0xEFF) == (255UL << 7)); // E=7, M=255
    std::cout << "DShot RPM codec host tests passed\n";
}
