// Portable host-side unit test. Run from repo root:
// g++ -std=c++17 -O2 -Wall -Wextra -Werror -IApplication tests/dshot_rpm_codec_test.cpp -o /tmp/dshot_rpm_test && /tmp/dshot_rpm_test
#include <cassert>
#include <cstdint>
#include <iostream>
#include "dshot_rpm_protocol.hpp"

namespace {
constexpr uint8_t GCR[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
};

// Independent, test-only waveform encoder (20-bit GCR plus 1 start bit).
uint32_t encode_reply(const uint16_t payload) {
    const uint16_t crc = static_cast<uint16_t>(~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0xF;
    const uint16_t frame = static_cast<uint16_t>((payload << 4) | crc);
    uint32_t gcr20 = 0;
    for (int i = 12; i >= 0; i -= 4) {
        gcr20 = (gcr20 << 5) | GCR[(frame >> i) & 0xF];
    }
    uint32_t raw21 = 0;
    uint32_t previous = 0;  // 21-bit symbol stream begins with zero
    for (int i = 19; i >= 0; --i) {
        previous ^= (gcr20 >> i) & 1U;
        raw21 = (raw21 << 1) | previous;
    }
    return raw21;  // omitted MSB is the leading zero
}
}

int main() {
    using namespace aim::ecat::dshot_rpm;
    uint16_t result = 0;
    for (uint16_t p = 0; p <= 0xFFF; ++p) {
        const bool allowed = p == STOPPED_PAYLOAD || (p & 0x1FF) != 0;
        const bool ok = decode_21bit(encode_reply(p), result);
        assert(ok == allowed);
        if (allowed) {
            assert(result == p);
        }
    }

    // Golden GCR vectors, generated from protocol coding table.
    assert(encode_reply(0x001) == 0x08BA4B);
    assert(encode_reply(0x401) == 0x0B45B3);
    assert(encode_reply(0xFFF) == 0x052951);

    assert(period_us(0x9F4) == 8000);
    assert(electrical_rpm(0x9F4) == 7500);
    assert(period_us(STOPPED_PAYLOAD) == 0);
    assert(electrical_rpm(STOPPED_PAYLOAD) == 0);

    // Invalid encoded GCR symbol and payload >12-bit never passes.
    assert(!decode_21bit(0, result));
    assert(!decode_21bit(0x200000, result));
    std::cout << "PASS: 4096 DShot eRPM payloads, CRC/GCR, golden vectors, timing" << std::endl;
}
