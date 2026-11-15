// Dust / airborne-particle filter for spinning LiDARs working on the range image. ROS-free.
//
// A dust echo is not part of a surface: its range disagrees with its angular neighbours (along the scan
// line and on the adjacent rings), it is usually weak, and it does not persist: a cloud of particles
// returns from a different place every sweep, while a surface (even a thin pole) returns from the same
// voxel again and again. Each cue alone fails somewhere (wet mud is as dark as dust, object silhouettes
// are range-discontinuous, moving vehicles do not persist), so a point is dust only when it is
// off-surface AND (weak OR strongly isolated) AND not seen as solid in the recent sweeps.
#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "iad_perception/types.hpp"

namespace iad::perception {

struct DustParams {
  bool use_intensity = true;
  bool use_geometry = true;
  bool use_temporal = true;
  double low_intensity = 12.0;  // weak echo threshold (sensor units)
  double very_low_intensity = 4.0;  // below this an echo is only kept on a clean surface (no relaxed rule, no memory)
  double range_tol_abs = 0.25, range_tol_rel = 0.03;
  int along_surface = 4;     // of the 6 scan-line neighbours, this many range-consistent = a surface
  int isolated_max = 1;      // <= this many consistent neighbours of 12 = strongly isolated (dust even if bright)
  double voxel = 0.6;        // persistence voxel [m] (map frame)
  int persist_min = 3;       // a voxel is solid if geometry confirmed a surface there in >= this many of the last 4 sweeps
};

class DustFilter {
 public:
  explicit DustFilter(const DustParams& p = {}) : p_(p) {}

  // `pts_map` = the scan's points in the map frame (for persistence). Returns 1 for dust.
  std::vector<uint8_t> classify(const Scan& s, const std::vector<Eigen::Vector3f>& pts_map) {
    const size_t n = s.pts.size();
    std::vector<uint8_t> dust(n, 0), surf(n, 0);
    ++sweep_;
    std::vector<float> range(n);
    for (size_t i = 0; i < n; ++i) {
      const auto& q = s.pts[i];
      range[i] = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    }
    // Neighbour relation: 1 = same surface (range within tol), 2 = behind (farther), 0 = missing or in
    // front (closer: an occluder, e.g. dust between the sensor and a wall, says nothing against the point).
    auto rel = [&](size_t i, int r, int c, double tol) {
      const int j = s.at(r, c);
      if (j < 0 || size_t(j) == i) return 0;
      const double d = range[size_t(j)] - range[i];
      return std::abs(d) < tol ? 1 : d > tol ? 2 : 0;
    };
    for (size_t i = 0; i < n; ++i) {
      const auto& q = s.pts[i];
      if (!p_.use_geometry) {
        dust[i] = p_.use_intensity && q.intensity < p_.low_intensity;
        continue;
      }
      const double tol = std::max(p_.range_tol_abs, p_.range_tol_rel * range[i]);
      int along = 0, same = 0, behind = 0;
      for (int da = -3; da <= 3; ++da) {
        if (!da) continue;
        const int k = rel(i, q.ring, q.col + da, tol);
        along += k == 1, same += k == 1, behind += k == 2;
      }
      for (int dr : {-1, 1})
        for (int da = -1; da <= 1; ++da) {
          const int k = rel(i, q.ring + dr, q.col + da, tol);
          same += k == 1, behind += k == 2;
        }
      // A surface continues along the scan line, or (silhouettes, surfaces seen through dust) continues on
      // at least one side along the line and agrees with at least as many neighbours as lie behind it.
      // Dust floats in front of whatever is behind it; with only sky behind, a dense cloud still lacks the
      // scan-line continuity.
      // Very weak echoes (dust backscatter) must be on a clean scan-line surface; the relaxed rule and the
      // persistence memory would let a dense, stationary plume through when the truck stands in it.
      const bool very_weak = p_.use_intensity && q.intensity < p_.very_low_intensity;
      const bool surface = along >= p_.along_surface || (!very_weak && along >= 2 && same >= 4 && same >= behind);
      if (surface) {
        surf[i] = 1;
        continue;
      }
      const bool weak = p_.use_intensity && q.intensity < p_.low_intensity;
      const bool isolated = same <= p_.isolated_max;
      if (!(weak || isolated)) continue;
      if (!very_weak && p_.use_temporal && !pts_map.empty() && solid(key(pts_map[i]))) continue;  // a persistent surface
      dust[i] = 1;
    }
    // Memory is fed by geometry-confirmed surface points only: a dust cloud hanging in one place never
    // becomes "solid", while a thin pole that is a clean surface in most sweeps does.
    if (p_.use_temporal && !pts_map.empty()) {
      for (size_t i = 0; i < n; ++i)
        if (surf[i]) {
          auto& h = hist_[key(pts_map[i])];
          if (h.last != sweep_) h.bits = (h.bits << (sweep_ - h.last >= 8 ? 8 : sweep_ - h.last)) | 1u, h.last = sweep_;
        }
      if (hist_.size() > 400000)
        for (auto it = hist_.begin(); it != hist_.end();) it = sweep_ - it->second.last > 4 ? hist_.erase(it) : std::next(it);
    }
    return dust;
  }

 private:
  int64_t key(const Eigen::Vector3f& p) const {
    const int64_t x = int64_t(std::floor(p.x() / p_.voxel)) & 0x1FFFFF, y = int64_t(std::floor(p.y() / p_.voxel)) & 0x1FFFFF;
    const int64_t z = int64_t(std::floor(p.z() / p_.voxel)) & 0x1FFFFF;
    return (x << 42) | (y << 21) | z;
  }
  struct Hist {
    uint32_t last = 0;
    uint8_t bits = 0;  // bit k: surface seen k sweeps before `last`
  };
  // Solid = confirmed in >= persist_min of the previous 4 sweeps (the current sweep is not counted).
  bool solid(int64_t k) const {
    auto it = hist_.find(k);
    if (it == hist_.end()) return false;
    const uint32_t age = sweep_ - it->second.last;  // >= 1 for previous sweeps
    if (age == 0 || age > 4) return false;
    const unsigned window = (unsigned(it->second.bits) << (age - 1)) & 0xF;
    return __builtin_popcount(window) >= p_.persist_min;
  }
  DustParams p_;
  uint32_t sweep_ = 0;
  std::unordered_map<int64_t, Hist> hist_;
};

}  // namespace iad::perception
