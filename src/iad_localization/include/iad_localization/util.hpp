// Localization-local helpers (this package shares no code with the other subsystems): math, voxel
// hashing, WGS84 <-> ENU, PCD map loading, PointCloud2 xyz/intensity I/O.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <string>

namespace iad::localization {


constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }
template <class T>
inline T sq(T x) { return x * x; }

// Planar pose of base_link (rear-axle centre on the ground), map frame.
struct Pose2 {
  double x = 0, y = 0, yaw = 0;
};

inline Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// ZYX (yaw-pitch-roll) Euler angles, returned as (roll, pitch, yaw).
inline Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q_in) {
  const Eigen::Quaterniond q = q_in.normalized();
  const double sinr = 2 * (q.w() * q.x() + q.y() * q.z()), cosr = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  const double sinp = std::clamp(2 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0);
  const double siny = 2 * (q.w() * q.z() + q.x() * q.y()), cosy = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  return {std::atan2(sinr, cosr), std::asin(sinp), std::atan2(siny, cosy)};
}

inline double yawFromQuat(const Eigen::Quaterniond& q) { return rpyFromQuat(q).z(); }

inline Eigen::Isometry3d makeIso(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = quatFromRPY(roll, pitch, yaw).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// Skew-symmetric matrix [v]x.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

// SE(3) exponential for a twist (omega, t) using the first-order (decoupled) approximation that is
// standard in point-cloud registration: R = exp([omega]x), translation applied directly.
inline Eigen::Isometry3d expSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d w = xi.head<3>();
  const double th = w.norm();
  T.linear() = th < 1e-12 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(th, w / th).toRotationMatrix();
  T.translation() = xi.tail<3>();
  return T;
}

// Rotation error angle [rad] and translation error [m] between two transforms.
inline std::pair<double, double> poseError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Isometry3d d = a.inverse() * b;
  return {Eigen::AngleAxisd(d.linear()).angle(), d.translation().norm()};
}

class Stopwatch {
 public:
  Stopwatch() : t0_(std::chrono::steady_clock::now()) {}
  double ms() const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_).count();
  }
  void reset() { t0_ = std::chrono::steady_clock::now(); }

 private:
  std::chrono::steady_clock::time_point t0_;
};



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


// ---- WGS84 geodetic <-> local ENU (exact, via ECEF)
struct GeoOrigin {
  double lat0 = 0, lon0 = 0, alt0 = 0;
  Eigen::Vector3d ecef0 = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // ECEF -> ENU
  GeoOrigin() = default;
  GeoOrigin(double lat, double lon, double alt) : lat0(lat), lon0(lon), alt0(alt) {
    ecef0 = ecef(lat, lon, alt);
    const double la = deg2rad(lat), lo = deg2rad(lon);
    R << -std::sin(lo), std::cos(lo), 0, -std::sin(la) * std::cos(lo), -std::sin(la) * std::sin(lo), std::cos(la),
        std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la);
  }
  static Eigen::Vector3d ecef(double lat, double lon, double alt) {
    const double a = 6378137.0, e2 = 6.69437999014e-3, la = deg2rad(lat), lo = deg2rad(lon);
    const double N = a / std::sqrt(1 - e2 * std::sin(la) * std::sin(la));
    return {(N + alt) * std::cos(la) * std::cos(lo), (N + alt) * std::cos(la) * std::sin(lo), (N * (1 - e2) + alt) * std::sin(la)};
  }
  Eigen::Vector3d toEnu(double lat, double lon, double alt) const { return R * (ecef(lat, lon, alt) - ecef0); }
};

// ---- point clouds
// Reads float x, y, z (+ intensity) from any PointCloud2 layout; drops non-finite points.
inline void readCloud(const sensor_msgs::msg::PointCloud2& m, std::vector<Eigen::Vector3d>& pts, std::vector<float>* inten = nullptr) {
  int ox = -1, oy = -1, oz = -1, oi = -1;
  for (const auto& f : m.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
    if (f.name == "x") ox = int(f.offset);
    else if (f.name == "y") oy = int(f.offset);
    else if (f.name == "z") oz = int(f.offset);
    else if (f.name == "intensity") oi = int(f.offset);
  }
  pts.clear();
  if (inten) inten->clear();
  if (ox < 0 || oy < 0 || oz < 0) return;
  const size_t n = size_t(m.width) * size_t(m.height);
  pts.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    const uint8_t* d = &m.data[k * m.point_step];
    float x, y, z, i = 0;
    std::memcpy(&x, d + ox, 4), std::memcpy(&y, d + oy, 4), std::memcpy(&z, d + oz, 4);
    if (oi >= 0) std::memcpy(&i, d + oi, 4);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    pts.emplace_back(x, y, z);
    if (inten) inten->push_back(i);
  }
}

inline sensor_msgs::msg::PointCloud2 cloudMsg(const std::vector<Eigen::Vector3d>& p, const std::vector<float>& intensity,
                                             const std::string& frame, const builtin_interfaces::msg::Time& st) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = frame, m.header.stamp = st, m.height = 1, m.width = uint32_t(p.size()), m.is_dense = true;
  const char* names[] = {"x", "y", "z", "intensity"};
  for (uint32_t k = 0; k < 4; ++k) {
    sensor_msgs::msg::PointField f;
    f.name = names[k], f.offset = 4 * k, f.datatype = sensor_msgs::msg::PointField::FLOAT32, f.count = 1;
    m.fields.push_back(f);
  }
  m.point_step = 16, m.row_step = 16 * m.width;
  m.data.resize(size_t(m.row_step));
  for (size_t i = 0; i < p.size(); ++i) {
    const float v[4] = {float(p[i].x()), float(p[i].y()), float(p[i].z()), i < intensity.size() ? intensity[i] : 0.f};
    std::memcpy(&m.data[i * 16], v, 16);
  }
  return m;
}

// PCD (ascii or binary, x y z [intensity], FLOAT32) -> points. Returns false on unsupported files.
inline bool loadPcd(const std::string& path, std::vector<Eigen::Vector3d>& pts, std::vector<float>* inten = nullptr) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::string line, data;
  std::vector<std::string> fields;
  size_t n = 0;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "FIELDS") {
      std::string x;
      while (ss >> x) fields.push_back(x);
    } else if (key == "POINTS") {
      ss >> n;
    } else if (key == "DATA") {
      ss >> data;
      break;
    }
  }
  int ix = -1, iy = -1, iz = -1, ii = -1;
  for (size_t k = 0; k < fields.size(); ++k) {
    if (fields[k] == "x") ix = int(k);
    else if (fields[k] == "y") iy = int(k);
    else if (fields[k] == "z") iz = int(k);
    else if (fields[k] == "intensity") ii = int(k);
  }
  if (ix < 0 || iy < 0 || iz < 0) return false;
  pts.clear(), pts.reserve(n);
  if (inten) inten->clear();
  std::vector<float> row(fields.size());
  for (size_t k = 0; k < n; ++k) {
    if (data == "binary") {
      f.read(reinterpret_cast<char*>(row.data()), std::streamsize(row.size() * 4));
    } else if (data == "ascii") {
      for (auto& v : row) f >> v;
    } else {
      return false;
    }
    if (!f) break;
    pts.emplace_back(row[size_t(ix)], row[size_t(iy)], row[size_t(iz)]);
    if (inten) inten->push_back(ii >= 0 ? row[size_t(ii)] : 0.f);
  }
  return !pts.empty();
}

}  // namespace iad::localization
