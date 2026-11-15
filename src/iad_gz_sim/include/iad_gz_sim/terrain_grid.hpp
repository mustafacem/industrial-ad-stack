// Reader for the terrain.grid file written by world_compiler (heights + surface materials), used by
// the gz plugins for traffic ground-following and surface-dependent rolling resistance.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace iad::gz {

// Material ids as in the world code (asphalt, gravel, dirt, mud, rock, concrete, water).
enum : uint8_t { kAsphalt = 0, kGravel, kDirt, kMud, kRock, kConcrete, kWater };

inline double rollingResistance(uint8_t m) {
  static const double k[] = {0.015, 0.030, 0.050, 0.100, 0.030, 0.012, 1.0};
  return m < 7 ? k[m] : 0.05;
}

struct TerrainGrid {
  int nx = 0, ny = 0;
  double res = 1, ox = 0, oy = 0;
  std::vector<float> h;
  std::vector<uint8_t> mat;

  bool load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char magic[8];
    f.read(magic, 8);
    if (std::string(magic, 6) != "IADTER") return false;
    int32_t dims[2];
    double geo[3];
    f.read(reinterpret_cast<char*>(dims), sizeof(dims));
    f.read(reinterpret_cast<char*>(geo), sizeof(geo));
    nx = dims[0], ny = dims[1], res = geo[0], ox = geo[1], oy = geo[2];
    h.resize(size_t(nx) * size_t(ny)), mat.resize(size_t(nx) * size_t(ny));
    f.read(reinterpret_cast<char*>(h.data()), std::streamsize(h.size() * sizeof(float)));
    f.read(reinterpret_cast<char*>(mat.data()), std::streamsize(mat.size()));
    return bool(f);
  }
  bool empty() const { return h.empty(); }
  float H(int i, int j) const {
    i = std::clamp(i, 0, nx - 1), j = std::clamp(j, 0, ny - 1);
    return h[size_t(j) * size_t(nx) + size_t(i)];
  }
  double height(double x, double y) const {
    if (empty()) return 0.0;
    const double fx = (x - ox) / res, fy = (y - oy) / res;
    const int i = int(std::floor(fx)), j = int(std::floor(fy));
    const double a = fx - i, b = fy - j;
    return (1 - a) * (1 - b) * H(i, j) + a * (1 - b) * H(i + 1, j) + (1 - a) * b * H(i, j + 1) + a * b * H(i + 1, j + 1);
  }
  uint8_t material(double x, double y) const {
    if (empty()) return kDirt;
    const int i = std::clamp(int(std::lround((x - ox) / res)), 0, nx - 1);
    const int j = std::clamp(int(std::lround((y - oy) / res)), 0, ny - 1);
    return mat[size_t(j) * size_t(nx) + size_t(i)];
  }
};

// Resolve "quarry/terrain.grid" against GZ_SIM_RESOURCE_PATH (where the compiled worlds live).
inline std::string findResource(const std::string& rel) {
  if (!rel.empty() && rel[0] == '/') return rel;
  const char* env = std::getenv("GZ_SIM_RESOURCE_PATH");
  std::stringstream ss(env ? env : "");
  std::string dir;
  while (std::getline(ss, dir, ':')) {
    if (dir.empty()) continue;
    const std::filesystem::path p = std::filesystem::path(dir) / rel;
    if (std::filesystem::exists(p)) return p.string();
  }
  return "";
}

}  // namespace iad::gz
