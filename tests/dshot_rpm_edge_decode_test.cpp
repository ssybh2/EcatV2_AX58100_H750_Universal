// Host-only synthetic timer-edge test for bidirectional DShot300 and DShot600 eRPM RX.
// The STM32 TIM/DMA driver is deliberately not assumed to be operational.
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <array>
#include "../Application/dshot_rpm_edge_decode.hpp"

namespace {
constexpr uint8_t GCR[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
};

uint32_t encode_reply(uint16_t payload) {
    uint16_t crc = static_cast<uint16_t>(~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0xF;
    const uint16_t word = static_cast<uint16_t>((payload << 4) | crc);
    uint32_t gcr = 0;
    for (int shift = 12; shift >= 0; shift -= 4) {
        gcr = (gcr << 5) | GCR[(word >> shift) & 0xF];
    }
    uint32_t result = 0;
    uint32_t state = 0;
    for (int bit = 19; bit >= 0; --bit) {
        state ^= ((gcr >> bit) & 1U);
        result = (result << 1) | state;
    }
    return result; // leading start bit is 0
}

struct CapturedEdges {
    std::array<uint32_t, 24> edges{};
    std::size_t length{0};
};

CapturedEdges make_edges(uint32_t raw, uint32_t start,
                          uint32_t bit_ticks = 16, int jitter = 0) {
    CapturedEdges result{};
    result.edges[result.length++] = start; // falling: idle 1 -> start bit 0
    uint32_t previous = 0;
    for (int bit = 1; bit < 21; ++bit) {
        const uint32_t level = (raw >> (20 - bit)) & 1U;
        if (level != previous) {
            const int error = jitter == 0 ? 0 : ((bit % 3) - 1) * jitter;
            result.edges[result.length++] =
                start + static_cast<uint32_t>(static_cast<int>(bit * bit_ticks) + error);
        }
        previous = level;
    }
    if (previous == 0) {
        // Some capture designs also record return to idle-high.
        result.edges[result.length++] = start + 21 * bit_ticks;
    }
    assert(result.length <= result.edges.size());
    return result;
}
}

int main() {
    using namespace aim::ecat::dshot_rpm;
    uint32_t success = 0;
    for (uint32_t p = 0; p <= 0xFFF; ++p) {
        const auto payload = static_cast<uint16_t>(p);
        const auto raw = encode_reply(payload);
        const auto edges = make_edges(raw, 0xFFFFFFF0UL, 16, 2);
        uint16_t decoded = 0;
        const bool result = decode_edge_timestamps(edges.edges.data(), edges.length, 16, decoded);
        const bool allowed = payload == STOPPED_PAYLOAD || (payload & 0x1FF) != 0;
        assert(result == allowed);
        if (allowed) {
            assert(decoded == payload);
            ++success;
        }
    }
    uint16_t unused = 0;
    assert(!decode_edge_timestamps(nullptr, 0, 16, unused));
    const auto good = make_edges(encode_reply(0x9F4), 1024);
    assert(decode_edge_timestamps(good.edges.data(), good.length, 16, unused));
    assert(unused == 0x9F4);
    // DShot300 ESC replies use 375 kbit/s; at 12 MHz that is 32 ticks/bit.
    const auto d300 = make_edges(encode_reply(0x9F4), 4096, 32, 3);
    assert(decode_edge_timestamps(d300.edges.data(), d300.length, 32, unused));
    assert(unused == 0x9F4);
    assert(!decode_edge_timestamps(d300.edges.data(), d300.length, 16, unused));
    // TIM3 has a 16-bit capture counter. Emulate 0xFFFF -> 0x0000 wrap.
    auto tim3 = make_edges(encode_reply(0x9F4), 0xFFF0, 16, 2);
    for (std::size_t i = 0; i < tim3.length; ++i) {
        tim3.edges[i] &= 0xFFFFU;
    }
    assert(decode_edge_timestamps(tim3.edges.data(), tim3.length, 16, unused, 0xFFFFU));
    assert(unused == 0x9F4);
    assert(!decode_edge_timestamps(good.edges.data(), good.length, 0, unused));
    auto bad = good;
    bad.edges[1] = bad.edges[0] + 1; // spurious edge / bouncing
    assert(!decode_edge_timestamps(bad.edges.data(), bad.length, 16, unused));
    std::cout << "PASS: " << success
              << " valid eRPM payloads decoded from jittered timer edges (including wraparound)\n";
}
