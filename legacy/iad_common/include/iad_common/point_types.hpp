// LiDAR point type used across the stack + PointCloud2 conversion.
// `label` is simulator ground truth; algorithms never read it, evaluators do.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace iad {

enum class ReturnType : uint8_t { kSingle = 0, kFirst = 1, kLast = 2 };
// kSteep: terrain steeper than 25 deg, or > 0.4 m above the lowest ground within 1.5 m (berm crests): not drivable.
// kObjectLow: object point < 0.3 m above the terrain under it (geometrically indistinguishable from ground).
enum class Label : uint8_t { kGround = 0, kStatic = 1, kDynamic = 2, kDust = 3, kEgo = 4, kSteep = 5, kObjectLow = 6, kUnknown = 255 };

#pragma pack(push, 1)
struct PointXYZIRL {
  float x, y, z, intensity;
  uint16_t ring;
  uint8_t return_type;
  uint8_t label;
};
#pragma pack(pop)
static_assert(sizeof(PointXYZIRL) == 20, "unexpected padding");

using Cloud = std::vector<PointXYZIRL>;

inline sensor_msgs::msg::PointCloud2 toMsg(const Cloud& pts, const std::string& frame,
                                           const builtin_interfaces::msg::Time& stamp) {
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.height = 1;
  m.width = uint32_t(pts.size());
  auto field = [](const char* n, uint32_t off, uint8_t type) {
    PointField f;
    f.name = n, f.offset = off, f.datatype = type, f.count = 1;
    return f;
  };
  m.fields = {field("x", 0, PointField::FLOAT32),         field("y", 4, PointField::FLOAT32),
              field("z", 8, PointField::FLOAT32),         field("intensity", 12, PointField::FLOAT32),
              field("ring", 16, PointField::UINT16),      field("return_type", 18, PointField::UINT8),
              field("label", 19, PointField::UINT8)};
  m.is_bigendian = false;
  m.point_step = sizeof(PointXYZIRL);
  m.row_step = m.point_step * m.width;
  m.is_dense = true;
  m.data.resize(pts.size() * sizeof(PointXYZIRL));
  if (!pts.empty()) std::memcpy(m.data.data(), pts.data(), m.data.size());
  return m;
}

// Reads any cloud with x/y/z float fields; optional intensity/ring/return_type/label.
inline Cloud fromMsg(const sensor_msgs::msg::PointCloud2& m) {
  using sensor_msgs::msg::PointField;
  int ox = -1, oy = -1, oz = -1, oi = -1, oring = -1, oret = -1, olab = -1;
  uint8_t ti = PointField::FLOAT32;
  for (const auto& f : m.fields) {
    if (f.name == "x") ox = int(f.offset);
    if (f.name == "y") oy = int(f.offset);
    if (f.name == "z") oz = int(f.offset);
    if (f.name == "intensity") oi = int(f.offset), ti = f.datatype;
    if (f.name == "ring") oring = int(f.offset);
    if (f.name == "return_type") oret = int(f.offset);
    if (f.name == "label") olab = int(f.offset);
  }
  const size_t n = size_t(m.width) * size_t(m.height);
  Cloud out;
  if (ox < 0 || oy < 0 || oz < 0) return out;
  if (m.point_step == sizeof(PointXYZIRL) && ox == 0 && oy == 4 && oz == 8 && oi == 12 && oring == 16 &&
      oret == 18 && olab == 19) {
    out.resize(n);
    std::memcpy(out.data(), m.data.data(), n * sizeof(PointXYZIRL));
    return out;
  }
  out.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    const uint8_t* p = m.data.data() + k * m.point_step;
    PointXYZIRL q{};
    std::memcpy(&q.x, p + ox, 4);
    std::memcpy(&q.y, p + oy, 4);
    std::memcpy(&q.z, p + oz, 4);
    if (oi >= 0) {
      if (ti == PointField::FLOAT32) std::memcpy(&q.intensity, p + oi, 4);
      else if (ti == PointField::UINT8) q.intensity = float(p[oi]);
    }
    if (oring >= 0) std::memcpy(&q.ring, p + oring, 2);
    q.return_type = oret >= 0 ? p[oret] : 0;
    q.label = olab >= 0 ? p[olab] : uint8_t(Label::kUnknown);
    if (std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z)) out.push_back(q);
  }
  return out;
}

}  // namespace iad
