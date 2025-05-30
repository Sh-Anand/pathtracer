// gpu_rand.h — ultra‑light, fast xorshift64*
// --------------------------------------------------------
// • 8 bytes state (s) instead of 16 bytes.
// • 3 xors + 3 shifts + 1 mul per 64‑bit draw.
// • SplitMix64 seeding for low cross‑seed correlation.
// • MIT licence – copy/paste freely.

#ifndef GPU_RAND_H
#define GPU_RAND_H

#include <stdint.h>
#include "cuda_defs.h"

// -----------------------------------------------------------------------------
// PRNG state (8 bytes)
// -----------------------------------------------------------------------------
struct RNGState {
    uint64_t s;
};

// -----------------------------------------------------------------------------
// SplitMix64 – only for seeding.
// -----------------------------------------------------------------------------
HOST_DEVICE inline static uint64_t splitmix64(uint64_t *x) {
    uint64_t z = (*x += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// -----------------------------------------------------------------------------
// Initialize per‑thread state from a 64‑bit seed.
// -----------------------------------------------------------------------------
HOST_DEVICE inline static void init_gpu_rng(RNGState *state, uint64_t seed) {
    // just one splitmix call to set state.s
    state->s = splitmix64(&seed);
    // avoid zero state (xorshift fails if s==0)
    if (state->s == 0) state->s = 0x9E3779B97F4A7C15ULL;
}

// -----------------------------------------------------------------------------
// xorshift64* outputting a full 64‑bit value.
// -----------------------------------------------------------------------------
HOST_DEVICE inline static uint64_t next_u64(RNGState *state) {
    uint64_t x = state->s;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    state->s = x;
    return x * 2685821657736338717ULL;
}

// -----------------------------------------------------------------------------
// Helpers for narrower outputs.
// -----------------------------------------------------------------------------
HOST_DEVICE inline static uint32_t next_u32(RNGState *state) {
    return static_cast<uint32_t>(next_u64(state) >> 32);
}

HOST_DEVICE inline static float next_float(RNGState *state) {
    // 24 MSBs → IEEE‑754 mantissa for [0,1)
    constexpr float INV_2_POW_24 = 1.0f / 16777216.0f;
    return static_cast<float>(next_u64(state) >> 40) * INV_2_POW_24;
}

#endif // GPU_RAND_H
