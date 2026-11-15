// Hash-based value noise / fBm used by world generation and visual albedo variation.
#pragma once
#include <cmath>
#include <cstdint>

namespace iad::sim {

inline uint32_t hash2(int x, int y, uint32_t seed) {
  uint32_t h = uint32_t(x) * 374761393u + uint32_t(y) * 668265263u + seed * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}
inline double lattice(int x, int y, uint32_t seed) { return (hash2(x, y, seed) & 0xFFFFFF) / double(0xFFFFFF) * 2.0 - 1.0; }
inline double smooth01(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
inline double vnoise(double x, double y, uint32_t seed) {
  const int xi = int(std::floor(x)), yi = int(std::floor(y));
  const double u = smooth01(x - xi), v = smooth01(y - yi);
  const double a = lattice(xi, yi, seed), b = lattice(xi + 1, yi, seed);
  const double c = lattice(xi, yi + 1, seed), d = lattice(xi + 1, yi + 1, seed);
  return (a + (b - a) * u) * (1 - v) + (c + (d - c) * u) * v;
}
inline double fbm(double x, double y, uint32_t seed, int oct = 4) {
  double s = 0, amp = 1, norm = 0;
  for (int k = 0; k < oct; ++k) {
    s += amp * vnoise(x, y, seed + uint32_t(k) * 101u), norm += amp;
    x *= 2.03, y *= 2.03, amp *= 0.5;
  }
  return s / norm;
}

}  // namespace iad::sim
