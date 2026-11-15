// Perception-local types and helpers (this package shares no code with the other subsystems).
// The LiDAR input is any sensor_msgs/PointCloud2 with float x, y, z (+ optional intensity, ring).
// An organized cloud (height = rings, width = azimuth columns, e.g. Gazebo gpu_lidar or a driver's
// "organized" mode) is used directly as a range image; an unorganized one is binned by ring/azimuth.
#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace iad::perception {

constexpr double kPi = 3.14159265358979323846;
inline double wrapAngle(double a) { return std::remainder(a, 2.0 * kPi); }

inline Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return {std::atan2(R(2, 1), R(2, 2)), std::asin(std::clamp(-R(2, 0), -1.0, 1.0)), std::atan2(R(1, 0), R(0, 0))};
}

struct Stopwatch {
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  void reset() { t0 = std::chrono::steady_clock::now(); }
  double ms() const { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); }
};

// Row-major cost grid with nav_msgs/OccupancyGrid semantics (-1 unknown, 0 free, 100 lethal).
struct CostGrid {
  int w = 0, h = 0;
  double res = 0.5, ox = 0, oy = 0;
  std::vector<int8_t> data;
};
constexpr int8_t kLethal = 100;
constexpr int8_t kUnknown = -1;

struct LidarPoint {
  float x = 0, y = 0, z = 0, intensity = 0;
  uint16_t ring = 0, col = 0;  // range-image row / column
};

struct Scan {
  std::vector<LidarPoint> pts;
  int rings = 0, cols = 0;
  std::vector<int> image;  // rings x cols -> point index or -1
  int at(int r, int c) const {
    if (r < 0 || r >= rings) return -1;
    c = ((c % cols) + cols) % cols;
    return image[size_t(r) * size_t(cols) + size_t(c)];
  }
};

namespace detail {
inline int fieldOffset(const sensor_msgs::msg::PointCloud2& m, const char* name, uint8_t& type) {
  for (const auto& f : m.fields)
    if (f.name == name) {
      type = f.datatype;
      return int(f.offset);
    }
  return -1;
}
}  // namespace detail

// Parse a PointCloud2 into a range image. Non-finite returns (no echo) are dropped.
inline Scan scanFromMsg(const sensor_msgs::msg::PointCloud2& m, int default_cols = 1200) {
  Scan s;
  uint8_t tx, ty, tz, ti = 0, tr = 0;
  const int ox = detail::fieldOffset(m, "x", tx), oy = detail::fieldOffset(m, "y", ty), oz = detail::fieldOffset(m, "z", tz);
  const int oi = detail::fieldOffset(m, "intensity", ti), orr = detail::fieldOffset(m, "ring", tr);
  if (ox < 0 || oy < 0 || oz < 0) return s;
  const size_t n = size_t(m.width) * size_t(m.height);
  const bool organized = m.height > 1;
  auto f32 = [&](size_t k, int off) {
    float v;
    std::memcpy(&v, &m.data[k * m.point_step + size_t(off)], 4);
    return v;
  };
  struct Raw {
    LidarPoint p;
    float elev;
  };
  std::vector<Raw> raw;
  raw.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    LidarPoint p;
    p.x = f32(k, ox), p.y = f32(k, oy), p.z = f32(k, oz);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (oi >= 0 && ti == sensor_msgs::msg::PointField::FLOAT32) p.intensity = f32(k, oi);
    if (organized) {
      p.ring = uint16_t(k / m.width), p.col = uint16_t(k % m.width);
    } else if (orr >= 0 && tr == sensor_msgs::msg::PointField::UINT16) {
      std::memcpy(&p.ring, &m.data[k * m.point_step + size_t(orr)], 2);
    }
    raw.push_back({p, float(std::atan2(p.z, std::hypot(p.x, p.y)))});
  }
  if (organized) {
    s.rings = int(m.height), s.cols = int(m.width);
  } else {
    s.cols = default_cols;
    if (orr < 0) {  // no ring field: bin the elevation angle into 0.5 deg rows
      float lo = 1e9f;
      for (const auto& r : raw) lo = std::min(lo, r.elev);
      for (auto& r : raw) r.p.ring = uint16_t(std::lround((r.elev - lo) / (0.5 * kPi / 180)));
    }
    for (auto& r : raw) {
      s.rings = std::max(s.rings, int(r.p.ring) + 1);
      const int c = int(std::floor((std::atan2(r.p.y, r.p.x) + kPi) / (2 * kPi) * s.cols));
      r.p.col = uint16_t(((c % s.cols) + s.cols) % s.cols);
    }
  }
  s.image.assign(size_t(std::max(s.rings, 1)) * size_t(std::max(s.cols, 1)), -1);
  s.pts.reserve(raw.size());
  for (const auto& r : raw) {
    s.image[size_t(r.p.ring) * size_t(s.cols) + r.p.col] = int(s.pts.size());
    s.pts.push_back(r.p);
  }
  return s;
}

// x, y, z, intensity (+ optional uint8 class) cloud.
inline sensor_msgs::msg::PointCloud2 cloudToMsg(const std::vector<Eigen::Vector3f>& p, const std::vector<float>& intensity,
                                               const std::string& frame, const builtin_interfaces::msg::Time& stamp,
                                               const std::vector<uint8_t>* cls = nullptr) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = frame, m.header.stamp = stamp;
  m.height = 1, m.width = uint32_t(p.size()), m.is_dense = true, m.is_bigendian = false;
  auto add = [&](const char* n, uint32_t off, uint8_t t) {
    sensor_msgs::msg::PointField f;
    f.name = n, f.offset = off, f.datatype = t, f.count = 1;
    m.fields.push_back(f);
  };
  add("x", 0, sensor_msgs::msg::PointField::FLOAT32), add("y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add("z", 8, sensor_msgs::msg::PointField::FLOAT32), add("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  m.point_step = 16;
  if (cls) add("class", 16, sensor_msgs::msg::PointField::UINT8), m.point_step = 20;
  m.row_step = m.point_step * m.width;
  m.data.resize(size_t(m.row_step));
  for (size_t i = 0; i < p.size(); ++i) {
    uint8_t* d = &m.data[i * m.point_step];
    std::memcpy(d, p[i].data(), 12);
    std::memcpy(d + 12, &intensity[i], 4);
    if (cls) d[16] = (*cls)[i];
  }
  return m;
}

}  // namespace iad::perception
