// Planning-local helpers (this package shares no code with the other subsystems): grids with
// nav_msgs/OccupancyGrid semantics, vehicle footprint, map-file loaders (PGM/YAML costmap, PCD elevation).
#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace iad::planning {

constexpr double kPi = 3.14159265358979323846;
inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double yawFromQuat(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

struct Stopwatch {
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  void reset() { t0 = std::chrono::steady_clock::now(); }
  double ms() const { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); }
};

struct Pose2 {
  double x = 0, y = 0, yaw = 0;
};


template <typename T>
struct Grid {
  int w = 0, h = 0;
  double res = 0.5, ox = 0, oy = 0;
  std::vector<T> data;

  Grid() = default;
  Grid(int w_, int h_, double r, double ox_, double oy_, T init = T{})
      : w(w_), h(h_), res(r), ox(ox_), oy(oy_), data(size_t(w_) * size_t(h_), init) {}

  bool inside(int i, int j) const { return i >= 0 && j >= 0 && i < w && j < h; }
  bool toCell(double x, double y, int& i, int& j) const {
    i = int(std::floor((x - ox) / res));
    j = int(std::floor((y - oy) / res));
    return inside(i, j);
  }
  double cx(int i) const { return ox + (i + 0.5) * res; }
  double cy(int j) const { return oy + (j + 0.5) * res; }
  T& at(int i, int j) { return data[size_t(j) * size_t(w) + size_t(i)]; }
  const T& at(int i, int j) const { return data[size_t(j) * size_t(w) + size_t(i)]; }
  T valueAt(double x, double y, T outside) const {
    int i, j;
    return toCell(x, y, i, j) ? at(i, j) : outside;
  }
  bool empty() const { return data.empty(); }
};

using CostGrid = Grid<int8_t>;
constexpr int8_t kLethal = 100;
constexpr int8_t kUnknown = -1;


// Vehicle geometry/kinematics the planner needs. base_link = rear-axle centre on the ground.
struct VehicleParams {
  double wheelbase = 4.3, width = 4.2, front = 6.6, rear = 2.8;  // [m]
  double max_steer = 0.6, max_steer_rate = 0.35;                 // [rad], [rad/s]
};

// Rectangle footprint (+ margin) vs cost grid: true if any sample lands on a lethal cell or off-map.
inline bool footprintCollides(const CostGrid& g, double x, double y, double yaw, const VehicleParams& v, double margin) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double len = v.front + v.rear + 2 * margin, wid = v.width + 2 * margin;
  const int na = int(std::ceil(len / (0.9 * g.res))) + 1, nb = int(std::ceil(wid / (0.9 * g.res))) + 1;
  for (int ia = 0; ia < na; ++ia) {
    const double a = -v.rear - margin + len * ia / (na - 1);
    for (int ib = 0; ib < nb; ++ib) {
      const double b = -wid / 2 + wid * ib / (nb - 1);
      int i, j;
      if (!g.toCell(x + a * c - b * s, y + a * s + b * c, i, j) || g.at(i, j) >= kLethal) return true;
    }
  }
  return false;
}

// nav2-style map YAML + PGM in "raw" mode (pixel = OccupancyGrid value, 255 = unknown) or "trinary".
inline bool loadCostmap(const std::string& yaml_path, CostGrid& g, std::string* err = nullptr) {
  std::ifstream y(yaml_path);
  if (!y) return err && (*err = "cannot open " + yaml_path, false);
  std::string line, image, mode = "trinary";
  double res = 0.5, ox = 0, oy = 0, occ = 0.65, fre = 0.196;
  int negate = 0;
  while (std::getline(y, line)) {
    auto val = [&]() { return line.substr(line.find(':') + 1); };
    auto trim = [](std::string s) { s.erase(0, s.find_first_not_of(" \t")); s.erase(s.find_last_not_of(" \t\r") + 1); return s; };
    if (line.rfind("image:", 0) == 0) image = trim(val());
    else if (line.rfind("mode:", 0) == 0) mode = trim(val());
    else if (line.rfind("resolution:", 0) == 0) res = std::stod(val());
    else if (line.rfind("negate:", 0) == 0) negate = std::stoi(val());
    else if (line.rfind("occupied_thresh:", 0) == 0) occ = std::stod(val());
    else if (line.rfind("free_thresh:", 0) == 0) fre = std::stod(val());
    else if (line.rfind("origin:", 0) == 0) {
      std::string v = line.substr(line.find('[') + 1);
      std::replace(v.begin(), v.end(), ',', ' ');
      std::istringstream ss(v);
      ss >> ox >> oy;
    }
  }
  if (image.empty()) return err && (*err = "no image in " + yaml_path, false);
  if (image[0] != '/') image = yaml_path.substr(0, yaml_path.find_last_of('/') + 1) + image;
  std::ifstream f(image, std::ios::binary);
  std::string magic;
  int w = 0, h = 0, maxv = 0;
  f >> magic >> w >> h >> maxv;
  f.get();
  if (magic != "P5" || w <= 0 || h <= 0) return err && (*err = "unsupported PGM " + image, false);
  std::vector<unsigned char> px(size_t(w) * size_t(h));
  f.read(reinterpret_cast<char*>(px.data()), std::streamsize(px.size()));
  g = CostGrid(w, h, res, ox, oy, 0);
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) {
      const int v = px[size_t(h - 1 - j) * size_t(w) + size_t(i)];  // PGM rows top-down, grid bottom-up
      int8_t c;
      if (mode == "raw") {
        c = v == 255 ? kUnknown : int8_t(std::min(v, 100));
      } else {
        const double p = negate ? v / 255.0 : (255.0 - v) / 255.0;
        c = p > occ ? kLethal : p < fre ? int8_t(0) : kUnknown;
      }
      g.at(i, j) = c;
    }
  return true;
}

// Ground elevation (lowest point per 1 m cell) from a PCD map, for trajectory z / grade. Optional.
inline bool loadElevation(const std::string& pcd, Grid<float>& elev) {
  std::ifstream f(pcd, std::ios::binary);
  if (!f) return false;
  std::string line, data;
  std::vector<std::string> fields;
  size_t n = 0;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string k;
    ss >> k;
    if (k == "FIELDS") for (std::string x; ss >> x;) fields.push_back(x);
    else if (k == "POINTS") ss >> n;
    else if (k == "DATA") { ss >> data; break; }
  }
  if (data != "binary" || fields.size() < 3) return false;
  std::vector<float> row(fields.size());
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(n);
  float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
  for (size_t k = 0; k < n && f.read(reinterpret_cast<char*>(row.data()), std::streamsize(row.size() * 4)); ++k) {
    pts.emplace_back(row[0], row[1], row[2]);
    x0 = std::min(x0, row[0]), y0 = std::min(y0, row[1]), x1 = std::max(x1, row[0]), y1 = std::max(y1, row[1]);
  }
  if (pts.empty()) return false;
  elev = Grid<float>(int(x1 - x0) + 2, int(y1 - y0) + 2, 1.0, x0, y0, 1e9f);
  for (const auto& p : pts) {
    int i, j;
    if (elev.toCell(p.x(), p.y(), i, j)) elev.at(i, j) = std::min(elev.at(i, j), p.z());
  }
  for (auto& v : elev.data)
    if (v > 1e8f) v = 0.f;
  return true;
}

}  // namespace iad::planning
