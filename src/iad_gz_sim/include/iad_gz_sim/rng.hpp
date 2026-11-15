// Small, fast, seedable RNG for per-thread sensor noise (std::mt19937 is too heavy to seed per ray column).
#pragma once
#include <cmath>
#include <cstdint>

namespace iad::sim {

struct FastRng {
  uint64_t s0 = 1, s1 = 2;
  bool has_spare = false;
  double spare = 0;

  explicit FastRng(uint64_t seed = 1) {
    uint64_t z = seed + 0x9E3779B97F4A7C15ull;
    s0 = mix(z);
    z += 0x9E3779B97F4A7C15ull;
    s1 = mix(z);
    if (!s0 && !s1) s1 = 1;
  }
  static uint64_t mix(uint64_t z) {
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  uint64_t next() {  // xorshift128+
    uint64_t x = s0;
    const uint64_t y = s1;
    s0 = y;
    x ^= x << 23;
    s1 = x ^ y ^ (x >> 17) ^ (y >> 26);
    return s1 + y;
  }
  double uniform() { return double(next() >> 11) * (1.0 / 9007199254740992.0); }
  double normal() {
    if (has_spare) return has_spare = false, spare;
    double u, v, s;
    do {
      u = 2 * uniform() - 1, v = 2 * uniform() - 1, s = u * u + v * v;
    } while (s >= 1 || s == 0);
    const double m = std::sqrt(-2 * std::log(s) / s);
    spare = v * m, has_spare = true;
    return u * m;
  }
};

}  // namespace iad::sim
