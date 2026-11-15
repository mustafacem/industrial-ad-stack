// Voxel hashing + centroid downsampling shared by perception, localization and calibration.
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace iad {

inline int64_t packVoxel(int ix, int iy, int iz) {
  constexpr int64_t kMask = (1 << 21) - 1;  // 21 bits per axis, two's complement wraps safely
  return ((int64_t(ix) & kMask) << 42) | ((int64_t(iy) & kMask) << 21) | (int64_t(iz) & kMask);
}

inline int64_t voxelOf(const Eigen::Vector3d& p, double inv_res) {
  return packVoxel(int(std::floor(p.x() * inv_res)), int(std::floor(p.y() * inv_res)),
                   int(std::floor(p.z() * inv_res)));
}

// Centroid of the points in each occupied voxel.
inline std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& pts, double res) {
  struct Acc { Eigen::Vector3d s = Eigen::Vector3d::Zero(); int n = 0; };
  std::unordered_map<int64_t, Acc> vox;
  vox.reserve(pts.size() / 2 + 1);
  const double inv = 1.0 / res;
  for (const auto& p : pts) {
    auto& a = vox[voxelOf(p, inv)];
    a.s += p, ++a.n;
  }
  std::vector<Eigen::Vector3d> out;
  out.reserve(vox.size());
  for (const auto& kv : vox) out.push_back(kv.second.s / kv.second.n);
  return out;
}

}  // namespace iad
