// Calibration-local helpers (this package shares no code with the other subsystems): math, voxel hashing,
// a voxel-plane (surfel) map for point-to-plane registration, PointCloud2 xyz I/O.
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace iad::calibration {

constexpr double kPi = 3.14159265358979323846;
inline double rad2deg(double r) { return r * 180.0 / kPi; }
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return {std::atan2(R(2, 1), R(2, 2)), std::asin(std::clamp(-R(2, 0), -1.0, 1.0)), std::atan2(R(1, 0), R(0, 0))};
}
inline Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}
inline Eigen::Isometry3d makeIso(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}
// Rotation angle [rad] and translation distance [m] between two transforms.
inline std::pair<double, double> poseError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Isometry3d d = a.inverse() * b;
  return {Eigen::AngleAxisd(d.linear()).angle(), d.translation().norm()};
}

inline int64_t packVoxel(int ix, int iy, int iz) {
  constexpr int64_t kMask = (1 << 21) - 1;
  return ((int64_t(ix) & kMask) << 42) | ((int64_t(iy) & kMask) << 21) | (int64_t(iz) & kMask);
}
inline int64_t voxelOf(const Eigen::Vector3d& p, double inv) {
  return packVoxel(int(std::floor(p.x() * inv)), int(std::floor(p.y() * inv)), int(std::floor(p.z() * inv)));
}
inline std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& pts, double res) {
  struct Acc { Eigen::Vector3d s = Eigen::Vector3d::Zero(); int n = 0; };
  std::unordered_map<int64_t, Acc> vox;
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

// Voxel-plane map of the reference LiDAR's accumulated cloud: per voxel the mean, the surface normal and
// the information along that normal only (a plane constrains nothing in-plane).
struct Surfel {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero(), normal = Eigen::Vector3d::UnitZ();
  Eigen::Matrix3d icov_normal = Eigen::Matrix3d::Zero();
};

class SurfelMap {
 public:
  void build(const std::vector<Eigen::Vector3d>& pts, double res, double min_std = 0.05, int min_pts = 5) {
    inv_ = 1.0 / res;
    struct Acc { Eigen::Vector3d s = Eigen::Vector3d::Zero(); Eigen::Matrix3d ss = Eigen::Matrix3d::Zero(); int n = 0; };
    std::unordered_map<int64_t, Acc> acc;
    for (const auto& p : pts) {
      auto& a = acc[voxelOf(p, inv_)];
      a.s += p, a.ss += p * p.transpose(), ++a.n;
    }
    vox_.clear();
    for (const auto& kv : acc) {
      const auto& a = kv.second;
      if (a.n < min_pts) continue;
      Surfel s;
      s.mean = a.s / a.n;
      const Eigen::Matrix3d C = (a.ss - a.n * s.mean * s.mean.transpose()) / (a.n - 1);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
      Eigen::Vector3d ev = es.eigenvalues();
      const double lmax = ev.maxCoeff();
      for (int k = 0; k < 3; ++k) ev[k] = std::max({ev[k], 0.01 * lmax, min_std * min_std});
      s.normal = es.eigenvectors().col(0);
      const bool planar = ev[0] * 10 < ev[1];
      s.icov_normal = planar ? Eigen::Matrix3d(s.normal * s.normal.transpose() / ev[0])
                             : Eigen::Matrix3d(es.eigenvectors() * ev.cwiseInverse().asDiagonal() * es.eigenvectors().transpose());
      vox_.emplace(kv.first, s);
    }
  }
  const Surfel* find(int ix, int iy, int iz) const {
    auto it = vox_.find(packVoxel(ix, iy, iz));
    return it == vox_.end() ? nullptr : &it->second;
  }
  double inv() const { return inv_; }
  size_t size() const { return vox_.size(); }

 private:
  std::unordered_map<int64_t, Surfel> vox_;
  double inv_ = 1.0;
};

inline std::vector<Eigen::Vector3d> readXyz(const sensor_msgs::msg::PointCloud2& m) {
  int ox = -1, oy = -1, oz = -1;
  for (const auto& f : m.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
    if (f.name == "x") ox = int(f.offset);
    else if (f.name == "y") oy = int(f.offset);
    else if (f.name == "z") oz = int(f.offset);
  }
  std::vector<Eigen::Vector3d> out;
  if (ox < 0 || oy < 0 || oz < 0) return out;
  const size_t n = size_t(m.width) * size_t(m.height);
  out.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    float x, y, z;
    const uint8_t* d = &m.data[k * m.point_step];
    std::memcpy(&x, d + ox, 4), std::memcpy(&y, d + oy, 4), std::memcpy(&z, d + oz, 4);
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) out.emplace_back(x, y, z);
  }
  return out;
}

inline sensor_msgs::msg::PointCloud2 xyzMsg(const std::vector<Eigen::Vector3d>& p, const std::string& frame,
                                           const builtin_interfaces::msg::Time& st) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = frame, m.header.stamp = st, m.height = 1, m.width = uint32_t(p.size()), m.is_dense = true;
  const char* names[] = {"x", "y", "z"};
  for (uint32_t k = 0; k < 3; ++k) {
    sensor_msgs::msg::PointField f;
    f.name = names[k], f.offset = 4 * k, f.datatype = sensor_msgs::msg::PointField::FLOAT32, f.count = 1;
    m.fields.push_back(f);
  }
  m.point_step = 12, m.row_step = 12 * m.width;
  m.data.resize(size_t(m.row_step));
  for (size_t i = 0; i < p.size(); ++i) {
    const float v[3] = {float(p[i].x()), float(p[i].y()), float(p[i].z())};
    std::memcpy(&m.data[i * 12], v, 12);
  }
  return m;
}

}  // namespace iad::calibration
