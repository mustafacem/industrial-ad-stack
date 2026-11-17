// World compiler: turns a seeded scenario (quarry | port) into a Gazebo Harmonic world plus the site
// map files the subsystems load (they stand in for a site survey):
//   <out>/world.sdf                     physics, lights, sensors systems, terrain, site geometry, traffic, dust
//   <out>/meshes/*.obj|mtl, *.png       terrain (one visual per surface material) + merged static geometry
//   <out>/terrain.grid                  heights + materials, read by the gz plugins (traffic z, rolling resistance)
//   <out>/maps/pointcloud_map.pcd       localization prior map
//   <out>/maps/costmap.pgm|yaml         planning site costmap (raw OccupancyGrid values)
//   <out>/site.yaml                     named poses (with z), routes, geodetic origin
//   <out>/extras/*.sdf                  optional models spawned by launch files (unmapped obstacle)
// Usage: world_compiler <quarry|port> <out_dir> [seed]
#include <zlib.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>

#include "iad_gz_sim/maps.hpp"
#include "iad_gz_sim/world.hpp"

namespace fs = std::filesystem;
using namespace iad;
using namespace iad::sim;

namespace {

// ------------------------------------------------------------------ small writers
void pngChunk(std::ofstream& f, const char* type, const std::vector<unsigned char>& data) {
  auto be32 = [&](uint32_t v) {
    const unsigned char b[4] = {uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)};
    f.write(reinterpret_cast<const char*>(b), 4);
  };
  be32(uint32_t(data.size()));
  f.write(type, 4);
  if (!data.empty()) f.write(reinterpret_cast<const char*>(data.data()), std::streamsize(data.size()));
  uLong crc = crc32(0L, reinterpret_cast<const Bytef*>(type), 4);
  if (!data.empty()) crc = crc32(crc, data.data(), uInt(data.size()));
  be32(uint32_t(crc));
}

void writePng(const fs::path& p, int w, int h, const std::vector<unsigned char>& rgb, int ch = 3) {
  std::vector<unsigned char> raw;
  raw.reserve(size_t(h) * (size_t(w) * size_t(ch) + 1));
  for (int y = 0; y < h; ++y) {
    raw.push_back(0);
    raw.insert(raw.end(), rgb.begin() + std::ptrdiff_t(size_t(y) * size_t(w) * size_t(ch)),
               rgb.begin() + std::ptrdiff_t(size_t(y + 1) * size_t(w) * size_t(ch)));
  }
  uLongf zlen = compressBound(uLong(raw.size()));
  std::vector<unsigned char> z(zlen);
  compress2(z.data(), &zlen, raw.data(), uLong(raw.size()), 6);
  z.resize(zlen);
  std::ofstream f(p, std::ios::binary);
  const unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  f.write(reinterpret_cast<const char*>(sig), 8);
  std::vector<unsigned char> ihdr = {uint8_t(w >> 24), uint8_t(w >> 16), uint8_t(w >> 8), uint8_t(w),
                                     uint8_t(h >> 24), uint8_t(h >> 16), uint8_t(h >> 8), uint8_t(h),
                                     8, uint8_t(ch == 4 ? 6 : 2), 0, 0, 0};
  pngChunk(f, "IHDR", ihdr);
  pngChunk(f, "IDAT", z);
  pngChunk(f, "IEND", {});
}

std::string f3(double v) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(3) << v;
  return s.str();
}
std::string col(const Color& c, double k = 1.0) {
  return f3(c[0] * k) + " " + f3(c[1] * k) + " " + f3(c[2] * k) + " 1";
}

// Value noise for texture variation.
struct Noise {
  int n = 256;
  std::vector<float> g;
  explicit Noise(uint32_t seed) : g(size_t(n) * size_t(n)) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> U(-1.f, 1.f);
    for (auto& v : g) v = U(rng);
  }
  float at(double x, double y) const {
    const int i = int(std::floor(x)), j = int(std::floor(y));
    const double fx = x - i, fy = y - j;
    auto G = [&](int a, int b) { return g[size_t((b & (n - 1)) * n + (a & (n - 1)))]; };
    const double sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
    return float((G(i, j) * (1 - sx) + G(i + 1, j) * sx) * (1 - sy) + (G(i, j + 1) * (1 - sx) + G(i + 1, j + 1) * sx) * sy);
  }
  float fbm(double x, double y) const { return 0.55f * at(x, y) + 0.3f * at(2.1 * x, 2.1 * y) + 0.15f * at(4.3 * x, 4.3 * y); }
};

// ------------------------------------------------------------------ mesh accumulation
struct Mesh {
  std::vector<Eigen::Vector3d> v, n;
  std::vector<Eigen::Vector2d> uv;
  std::vector<std::array<int, 3>> f;
  int add(const Eigen::Vector3d& p, const Eigen::Vector3d& nn, const Eigen::Vector2d& t = Eigen::Vector2d::Zero()) {
    v.push_back(p), n.push_back(nn), uv.push_back(t);
    return int(v.size()) - 1;
  }
  void quad(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d) {
    const Eigen::Vector3d nn = (b - a).cross(c - a).normalized();
    const int i0 = add(a, nn), i1 = add(b, nn), i2 = add(c, nn), i3 = add(d, nn);
    f.push_back({i0, i1, i2}), f.push_back({i0, i2, i3});
  }
  void box(const Box& b) {
    const double c = std::cos(b.yaw), s = std::sin(b.yaw);
    auto P = [&](double sx, double sy, double sz) {
      const Eigen::Vector3d l(sx * b.half.x(), sy * b.half.y(), sz * b.half.z());
      return Eigen::Vector3d(b.c.x() + c * l.x() - s * l.y(), b.c.y() + s * l.x() + c * l.y(), b.c.z() + l.z());
    };
    quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1));      // top
    quad(P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), P(-1, -1, -1));  // bottom
    quad(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1));      // +x
    quad(P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1));  // -x
    quad(P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1));      // +y
    quad(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1));  // -y
  }
  void cylinder(const Cylinder& cy, int seg = 18) {
    for (int k = 0; k < seg; ++k) {
      const double a0 = 2 * kPi * k / seg, a1 = 2 * kPi * (k + 1) / seg;
      const Eigen::Vector3d p0(cy.x + cy.r * std::cos(a0), cy.y + cy.r * std::sin(a0), 0);
      const Eigen::Vector3d p1(cy.x + cy.r * std::cos(a1), cy.y + cy.r * std::sin(a1), 0);
      const Eigen::Vector3d n0(std::cos(a0), std::sin(a0), 0), n1(std::cos(a1), std::sin(a1), 0);
      const Eigen::Vector3d z0(0, 0, cy.z0), z1(0, 0, cy.z1);
      const int i0 = add(p0 + z0, n0), i1 = add(p1 + z0, n1), i2 = add(p1 + z1, n1), i3 = add(p0 + z1, n0);
      f.push_back({i0, i1, i2}), f.push_back({i0, i2, i3});
      const Eigen::Vector3d up(0, 0, 1), ctr(cy.x, cy.y, cy.z1);
      const int c0 = add(ctr, up), c1 = add(p0 + z1, up), c2 = add(p1 + z1, up);
      f.push_back({c0, c1, c2});
    }
  }
  // Low-poly boulder: jittered icosphere scaled to the rock's footprint and height.
  void boulder(const Cylinder& cy, uint32_t seed) {
    static const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    std::vector<Eigen::Vector3d> ico = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                                        {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    std::vector<std::array<int, 3>> tri = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                                           {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8},
                                           {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(0.82, 1.12);
    std::vector<double> jit(ico.size());
    for (auto& j : jit) j = U(rng);
    const double hz = 0.5 * (cy.z1 - cy.z0);
    const Eigen::Vector3d c(cy.x, cy.y, cy.z0 + hz);
    auto P = [&](int i) {
      const Eigen::Vector3d u = ico[size_t(i)].normalized() * jit[size_t(i)];
      return Eigen::Vector3d(c.x() + cy.r * u.x(), c.y() + cy.r * u.y(), c.z() + hz * u.z());
    };
    for (auto& tr : tri) {
      const Eigen::Vector3d a = P(tr[0]), b = P(tr[1]), d = P(tr[2]);
      const Eigen::Vector3d nn = (b - a).cross(d - a).normalized();
      const int i0 = add(a, nn), i1 = add(b, nn), i2 = add(d, nn);
      f.push_back({i0, i1, i2});
    }
  }
  void writeObj(const fs::path& p, const std::string& mtl_file, const std::string& mtl_name, bool with_uv) const {
    std::ofstream o(p);
    o << std::fixed << std::setprecision(3) << "mtllib " << mtl_file << "\nusemtl " << mtl_name << "\n";
    for (auto& x : v) o << "v " << x.x() << " " << x.y() << " " << x.z() << "\n";
    o << std::setprecision(4);
    for (auto& x : n) o << "vn " << x.x() << " " << x.y() << " " << x.z() << "\n";
    if (with_uv) {
      o << std::setprecision(5);
      for (auto& x : uv) o << "vt " << x.x() << " " << x.y() << "\n";
      for (auto& t : f)
        o << "f " << t[0] + 1 << "/" << t[0] + 1 << "/" << t[0] + 1 << " " << t[1] + 1 << "/" << t[1] + 1 << "/" << t[1] + 1
          << " " << t[2] + 1 << "/" << t[2] + 1 << "/" << t[2] + 1 << "\n";
    } else {
      for (auto& t : f) o << "f " << t[0] + 1 << "//" << t[0] + 1 << " " << t[1] + 1 << "//" << t[1] + 1 << " " << t[2] + 1 << "//" << t[2] + 1 << "\n";
    }
  }
};

void writeMtl(const fs::path& p, const std::string& name, const Color& c, const std::string& tex = "") {
  std::ofstream o(p);
  o << "newmtl " << name << "\nKa " << f3(c[0]) << " " << f3(c[1]) << " " << f3(c[2]) << "\nKd " << f3(c[0]) << " "
    << f3(c[1]) << " " << f3(c[2]) << "\nKs 0.05 0.05 0.05\nNs 10\nd 1\nillum 2\n";
  if (!tex.empty()) o << "map_Kd " << tex << "\n";
}

const char* matName(uint8_t m) {
  static const char* k[] = {"asphalt", "gravel", "dirt", "mud", "rock", "concrete", "water"};
  return m < kNumMaterials ? k[m] : "dirt";
}

double retro(float refl) { return std::round(100.0 * refl); }

// ------------------------------------------------------------------ terrain
// Grid mesh of the terrain at `step`, triangles grouped by the material of their cell, sharing one
// albedo texture. Each group becomes its own visual so the LiDAR intensity (laser_retro) follows the
// surface.
std::map<uint8_t, Mesh> terrainMeshes(const Terrain& t, double step, bool split) {
  const int k = std::max(1, int(std::lround(step / t.res)));
  const int nx = (t.nx - 1) / k + 1, ny = (t.ny - 1) / k + 1;
  const double W = (t.nx - 1) * t.res, H = (t.ny - 1) * t.res;
  std::map<uint8_t, Mesh> out;
  std::map<uint8_t, std::vector<int>> idx;  // vertex index per grid node, per group (-1 = not yet added)
  auto node = [&](uint8_t g, int i, int j) {
    auto& ix = idx[g];
    if (ix.empty()) ix.assign(size_t(nx) * size_t(ny), -1);
    int& id = ix[size_t(j) * size_t(nx) + size_t(i)];
    if (id < 0) {
      const double x = t.xOf(i * k), y = t.yOf(j * k);
      id = out[g].add(Eigen::Vector3d(x, y, t.H(i * k, j * k)), t.normal(x, y), Eigen::Vector2d((x - t.ox) / W, 1.0 - (y - t.oy) / H));
    }
    return id;
  };
  for (int j = 0; j + 1 < ny; ++j)
    for (int i = 0; i + 1 < nx; ++i) {
      const double cx = t.xOf(i * k) + 0.5 * step, cy = t.yOf(j * k) + 0.5 * step;
      const uint8_t g = split ? t.material(cx, cy) : 0;
      const int a = node(g, i, j), b = node(g, i + 1, j), c = node(g, i + 1, j + 1), d = node(g, i, j + 1);
      out[g].f.push_back({a, b, c}), out[g].f.push_back({a, c, d});
    }
  return out;
}

void writeTerrainTexture(const World& w, const fs::path& p, double texel) {
  const Terrain& t = w.terrain;
  const double W = (t.nx - 1) * t.res, H = (t.ny - 1) * t.res;
  const int tw = int(W / texel), th = int(H / texel);
  std::vector<unsigned char> img(size_t(tw) * size_t(th) * 3);
  Noise nz(17);
  for (int y = 0; y < th; ++y)
    for (int x = 0; x < tw; ++x) {
      const double wx = t.ox + (x + 0.5) * texel, wy = t.oy + H - (y + 0.5) * texel;
      // Bilinear blend of the four surrounding material colours: no staircase at road edges.
      const uint8_t m = t.material(wx, wy);
      const double fx = (wx - t.ox) / t.res, fy = (wy - t.oy) / t.res;
      const int i0 = int(std::floor(fx)), j0 = int(std::floor(fy));
      const double ax = fx - i0, ay = fy - j0;
      std::array<double, 3> c{0, 0, 0};
      for (int dj = 0; dj < 2; ++dj)
        for (int di = 0; di < 2; ++di) {
          const int ii = std::clamp(i0 + di, 0, t.nx - 1), jj = std::clamp(j0 + dj, 0, t.ny - 1);
          const auto& mc = materialInfo(t.mat[size_t(jj) * size_t(t.nx) + size_t(ii)]).color;
          const double wgt = (di ? ax : 1 - ax) * (dj ? ay : 1 - ay);
          for (int ch = 0; ch < 3; ++ch) c[size_t(ch)] += wgt * mc[size_t(ch)];
        }
      const double fine = nz.fbm(wx * 1.7, wy * 1.7), coarse = nz.fbm(wx * 0.08 + 31, wy * 0.08 + 7);
      double k = 1.0 + 0.10 * fine + 0.08 * coarse;
      if (m == kWater) k = 1.0 + 0.03 * fine;
      if (m == kAsphalt || m == kConcrete) k = 1.0 + 0.05 * fine + 0.04 * coarse;
      // Steep faces (berm flanks, cut walls) a little darker.
      if (t.steep(wx, wy) && m != kWater) k *= 0.92;
      for (int ch = 0; ch < 3; ++ch)
        img[(size_t(y) * size_t(tw) + size_t(x)) * 3 + size_t(ch)] = uint8_t(std::clamp(255.0 * c[size_t(ch)] * k, 0.0, 255.0));
    }
  writePng(p, tw, th, img);
}

// Soft dust sprite (radial alpha with noise) + a colour/alpha ramp over the particle lifetime.
void writeDustTextures(const fs::path& dir) {
  const int n = 128;
  std::vector<unsigned char> img(size_t(n) * n * 4);
  Noise nz(5);
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x) {
      const double u = (x + 0.5) / n * 2 - 1, v = (y + 0.5) / n * 2 - 1, r2 = u * u + v * v;
      const double a = std::max(0.0, 1.0 - r2) * std::max(0.0, 1.0 - r2) * (0.7 + 0.3 * nz.fbm(x * 0.12, y * 0.12));
      unsigned char* q = &img[(size_t(y) * n + size_t(x)) * 4];
      q[0] = 214, q[1] = 196, q[2] = 168, q[3] = uint8_t(std::clamp(255.0 * a, 0.0, 255.0));
    }
  writePng(dir / "dust.png", n, n, img, 4);
  const int m = 64;
  std::vector<unsigned char> ramp(size_t(m) * 4);
  for (int x = 0; x < m; ++x) {
    const double t = double(x) / (m - 1), a = t < 0.15 ? t / 0.15 : 1.0 - (t - 0.15) / 0.85;
    ramp[size_t(x) * 4 + 0] = uint8_t(196 + 30 * t), ramp[size_t(x) * 4 + 1] = uint8_t(176 + 32 * t);
    ramp[size_t(x) * 4 + 2] = uint8_t(150 + 36 * t), ramp[size_t(x) * 4 + 3] = uint8_t(150 * std::max(0.0, a));
  }
  writePng(dir / "dust_colors.png", m, 1, ramp, 4);
}

void writeTerrainGrid(const Terrain& t, const fs::path& p) {
  std::ofstream o(p, std::ios::binary);
  const char magic[8] = {'I', 'A', 'D', 'T', 'E', 'R', '1', 0};
  o.write(magic, 8);
  const int32_t dims[2] = {t.nx, t.ny};
  const double geo[3] = {t.res, t.ox, t.oy};
  o.write(reinterpret_cast<const char*>(dims), sizeof(dims));
  o.write(reinterpret_cast<const char*>(geo), sizeof(geo));
  o.write(reinterpret_cast<const char*>(t.h.data()), std::streamsize(t.h.size() * sizeof(float)));
  o.write(reinterpret_cast<const char*>(t.mat.data()), std::streamsize(t.mat.size()));
}

// ------------------------------------------------------------------ maps
void writePcd(const Cloud& c, const fs::path& p) {
  std::ofstream o(p, std::ios::binary);
  o << "# .PCD v0.7 - iad_gz_sim world compiler\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
    << "COUNT 1 1 1 1\nWIDTH " << c.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << c.size() << "\nDATA binary\n";
  for (const auto& q : c) {
    const float v[4] = {q.x, q.y, q.z, q.intensity};
    o.write(reinterpret_cast<const char*>(v), sizeof(v));
  }
}

void writeCostmap(const CostGrid& g, const fs::path& dir) {
  // nav2 map_server "raw" mode: the pixel value is the OccupancyGrid value (255 = unknown).
  // PGM rows go top-down, the grid goes bottom-up.
  std::ofstream o(dir / "costmap.pgm", std::ios::binary);
  o << "P5\n" << g.w << " " << g.h << "\n255\n";
  for (int j = g.h - 1; j >= 0; --j)
    for (int i = 0; i < g.w; ++i) {
      const int8_t v = g.at(i, j);
      const unsigned char b = v < 0 ? 255 : uint8_t(v);
      o.write(reinterpret_cast<const char*>(&b), 1);
    }
  std::ofstream y(dir / "costmap.yaml");
  y << "image: costmap.pgm\nmode: raw\nresolution: " << g.res << "\norigin: [" << g.ox << ", " << g.oy
    << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
}

// ------------------------------------------------------------------ SDF pieces
std::string poseStr(double x, double y, double z, double roll, double pitch, double yaw) {
  return "<pose>" + f3(x) + " " + f3(y) + " " + f3(z) + " " + f3(roll) + " " + f3(pitch) + " " + f3(yaw) + "</pose>";
}

std::string material(const Color& c, double emissive = 0.0) {
  std::string s = "<material><ambient>" + col(c, 0.7) + "</ambient><diffuse>" + col(c) + "</diffuse><specular>0.08 0.08 0.08 1</specular>";
  if (emissive > 0) s += "<emissive>" + col(c, emissive) + "</emissive>";
  return s + "</material>";
}

std::string meshVisual(const std::string& name, const std::string& uri, double retro_v, const std::string& mat = "") {
  return "<visual name=\"" + name + "\"><cast_shadows>true</cast_shadows><laser_retro>" + f3(retro_v) +
         "</laser_retro><geometry><mesh><uri>" + uri + "</uri></mesh></geometry>" + mat + "</visual>\n";
}

std::string dustMaterial(const std::string& uri) {
  return "<material><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal><albedo_map>" + uri +
         "/meshes/dust.png</albedo_map></metal></pbr></material><color_range_image>" + uri +
         "/meshes/dust_colors.png</color_range_image>";
}

struct Group {
  Mesh mesh;
  Color color;
  float refl;
};

std::string key(const Color& c, float refl) {
  std::ostringstream s;
  s << int(c[0] * 50) << "_" << int(c[1] * 50) << "_" << int(c[2] * 50) << "_" << int(refl * 20);
  return s.str();
}

// Lat/lon of the site origin (world (0,0)) for the NavSat sensor; realistic places, nothing else implied.
void geoOrigin(const std::string& name, double& lat, double& lon) {
  if (name == "port") lat = 35.6206, lon = 139.7900;  // Tokyo Bay waterfront
  else lat = 35.9900, lon = 139.0800;                 // Chichibu limestone quarry area
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: world_compiler <quarry|port> <out_dir> [seed]\n";
    return 2;
  }
  const std::string name = argv[1];
  const fs::path out = argv[2];
  const uint32_t seed = argc > 3 ? uint32_t(std::stoul(argv[3])) : 7u;
  World w = makeWorld(name, seed);
  const std::string uri = "model://" + name;  // <out> is found through GZ_SIM_RESOURCE_PATH
  fs::create_directories(out / "meshes"), fs::create_directories(out / "maps"), fs::create_directories(out / "extras");
  const Terrain& t = w.terrain;

  // ---- terrain: visuals per material (0.5 m quarry / 1 m port), one collision mesh (1 m) + mud (low friction)
  writeTerrainTexture(w, out / "meshes" / "terrain_albedo.png", name == "port" ? 0.25 : 0.2);
  std::ostringstream terrain;
  terrain << "<model name=\"terrain\"><static>true</static><link name=\"link\">\n";
  const double vstep = name == "port" ? 1.0 : 0.5;
  for (auto& [m, mesh] : terrainMeshes(t, vstep, true)) {
    const std::string base = std::string("terrain_") + matName(m);
    writeMtl(out / "meshes" / (base + ".mtl"), base, {1, 1, 1, 1}, "terrain_albedo.png");
    mesh.writeObj(out / "meshes" / (base + ".obj"), base + ".mtl", base, true);
    std::string extra;
    if (m == kWater) extra = "<material><diffuse>0.09 0.2 0.3 1</diffuse><specular>0.6 0.6 0.6 1</specular></material>";
    terrain << meshVisual(base, uri + "/meshes/" + base + ".obj", retro(materialInfo(m).reflectivity), extra);
  }
  {
    auto coll = terrainMeshes(t, 1.0, true);
    Mesh hard, mud;
    for (auto& [m, mesh] : coll) {
      Mesh& dst = m == kMud ? mud : hard;
      const int off = int(dst.v.size());
      dst.v.insert(dst.v.end(), mesh.v.begin(), mesh.v.end());
      dst.n.insert(dst.n.end(), mesh.n.begin(), mesh.n.end());
      dst.uv.insert(dst.uv.end(), mesh.uv.begin(), mesh.uv.end());
      for (auto f : mesh.f) dst.f.push_back({f[0] + off, f[1] + off, f[2] + off});
    }
    auto collision = [&](const std::string& n, const Mesh& m, double mu) {
      if (m.f.empty()) return;
      writeMtl(out / "meshes" / (n + ".mtl"), n, {0.5f, 0.5f, 0.5f, 1});
      m.writeObj(out / "meshes" / (n + ".obj"), n + ".mtl", n, false);
      terrain << "<collision name=\"" << n << "\"><geometry><mesh><uri>" << uri << "/meshes/" << n
              << ".obj</uri></mesh></geometry><surface><friction><ode><mu>" << mu << "</mu><mu2>" << mu
              << "</mu2></ode></friction><contact><collide_bitmask>0xffff</collide_bitmask></contact></surface></collision>\n";
    };
    collision("terrain_collision", hard, 0.9);
    collision("terrain_collision_mud", mud, 0.32);
  }
  terrain << "</link></model>\n";
  writeTerrainGrid(t, out / "terrain.grid");
  writeDustTextures(out / "meshes");

  // ---- static site geometry: visuals merged per colour (cheap to render), primitive collisions
  std::map<std::string, Group> groups;
  std::ostringstream coll;
  int nb = 0, nc = 0;
  auto group = [&](const Color& c, float refl) -> Mesh& {
    auto& g = groups[key(c, refl)];
    g.color = c, g.refl = refl;
    return g.mesh;
  };
  for (const auto& b : w.boxes) {
    if (b.visible) group(b.color, b.refl).box(b);
    coll << "<collision name=\"b" << nb++ << "\">" << poseStr(b.c.x(), b.c.y(), b.c.z(), 0, 0, b.yaw)
         << "<geometry><box><size>" << f3(2 * b.half.x()) << " " << f3(2 * b.half.y()) << " " << f3(2 * b.half.z())
         << "</size></box></geometry></collision>\n";
  }
  for (const auto& b : w.visuals) group(b.color, b.refl).box(b);
  uint32_t rock_seed = 1;
  for (const auto& c : w.cylinders) {
    if (c.rock) {
      group(c.color, c.refl).boulder(c, rock_seed++);
      coll << "<collision name=\"c" << nc++ << "\">" << poseStr(c.x, c.y, 0.5 * (c.z0 + c.z1), 0, 0, 0)
           << "<geometry><ellipsoid><radii>" << f3(0.9 * c.r) << " " << f3(0.9 * c.r) << " " << f3(0.5 * (c.z1 - c.z0))
           << "</radii></ellipsoid></geometry></collision>\n";
    } else {
      group(c.color, c.refl).cylinder(c);
      coll << "<collision name=\"c" << nc++ << "\">" << poseStr(c.x, c.y, 0.5 * (c.z0 + c.z1), 0, 0, 0)
           << "<geometry><cylinder><radius>" << f3(c.r) << "</radius><length>" << f3(c.z1 - c.z0)
           << "</length></cylinder></geometry></collision>\n";
    }
  }
  std::ostringstream site;
  site << "<model name=\"site\"><static>true</static><link name=\"link\">\n" << coll.str();
  int gi = 0;
  for (auto& [k, g] : groups) {
    const std::string n = "site_" + std::to_string(gi++);
    writeMtl(out / "meshes" / (n + ".mtl"), n, g.color);
    g.mesh.writeObj(out / "meshes" / (n + ".obj"), n + ".mtl", n, false);
    site << meshVisual(n, uri + "/meshes/" + n + ".obj", retro(g.refl), material(g.color));
  }
  site << "</link></model>\n";

  // ---- traffic: scripted kinematic models (visual only), moved by iad_gz_sim::Traffic
  std::ostringstream traffic;
  int ai = 0;
  for (const auto& a : w.actors) {
    const std::string n = a.kind + "_" + std::to_string(ai++);
    traffic << "<model name=\"" << n << "\"><static>false</static>" << poseStr(a.path[0].x(), a.path[0].y(), t.height(a.path[0].x(), a.path[0].y()), 0, 0, 0)
            << "<link name=\"link\"><gravity>false</gravity><inertial><mass>1000</mass></inertial>\n";
    int pi = 0;
    for (const auto& p : a.parts)
      traffic << "<visual name=\"p" << pi++ << "\">" << poseStr(p.c.x(), p.c.y(), p.c.z(), 0, 0, p.yaw) << "<laser_retro>"
              << f3(retro(p.refl)) << "</laser_retro><geometry><box><size>" << f3(2 * p.half.x()) << " " << f3(2 * p.half.y())
              << " " << f3(2 * p.half.z()) << "</size></box></geometry>" << material(p.color) << "</visual>\n";
    if (a.emits_dust)
      traffic << "<particle_emitter name=\"dust\" type=\"box\"><emitting>false</emitting>" << poseStr(-4.5, 0, 1.0, 0, 0, 0)
              << "<size>3 3 1.5</size><particle_size>1.2 1.2 1.2</particle_size><lifetime>5</lifetime><rate>25</rate>"
              << "<min_velocity>0.2</min_velocity><max_velocity>0.8</max_velocity><scale_rate>0.9</scale_rate>"
              << dustMaterial(uri)
              << "<particle_scatter_ratio>0.08</particle_scatter_ratio></particle_emitter>\n";
    std::ostringstream wp;
    for (const auto& p : a.path) wp << f3(p.x()) << " " << f3(p.y()) << " ";
    double front = 0, half_w = 0;  // footprint from the parts (local frame: x forward from the origin)
    for (const auto& p : a.parts) {
      const double c = std::abs(std::cos(p.yaw)), sn = std::abs(std::sin(p.yaw));
      front = std::max(front, p.c.x() + p.half.x() * c + p.half.y() * sn);
      half_w = std::max(half_w, std::abs(p.c.y()) + p.half.x() * sn + p.half.y() * c);
    }
    traffic << "</link><plugin filename=\"iad_gz_traffic\" name=\"iad_gz_sim::Traffic\"><terrain_grid>" << name
            << "/terrain.grid</terrain_grid><waypoints>" << wp.str() << "</waypoints><speed>" << a.speed << "</speed><loop>"
            << (a.loop ? 1 : 0) << "</loop><bidirectional>" << (a.bidirectional ? 1 : 0) << "</bidirectional><wait>"
            << a.wait_at_end << "</wait><start_s>" << a.s << "</start_s><start_dir>" << a.dir << "</start_dir><dust>"
            << (a.emits_dust ? 1 : 0) << "</dust><front>" << f3(front) << "</front><half_width>" << f3(half_w)
            << "</half_width></plugin></model>\n";
  }

  // ---- persistent dust sources (dump face, crusher, wind-blown road)
  std::ostringstream dust;
  int di = 0;
  for (const auto& s : w.dust_sources) {
    const double rate = std::clamp(25.0 * s.rate * s.density / 0.15, 6.0, 60.0);
    dust << "<model name=\"dust_" << di++ << "\"><static>true</static>" << poseStr(s.c.x(), s.c.y(), s.c.z(), 0, 0, 0)
         << "<link name=\"link\"><particle_emitter name=\"emitter\" type=\"box\"><emitting>true</emitting><size>"
         << f3(s.spread * 1.4) << " " << f3(s.spread * 1.4) << " " << f3(s.radius * 0.6) << "</size><particle_size>"
         << f3(0.5 * s.radius) << " " << f3(0.5 * s.radius) << " " << f3(0.5 * s.radius) << "</particle_size>"
         << "<lifetime>9</lifetime><rate>" << f3(rate) << "</rate><min_velocity>0.2</min_velocity><max_velocity>0.9</max_velocity>"
         << "<scale_rate>0.5</scale_rate>" << dustMaterial(uri)
         << "<particle_scatter_ratio>" << f3(std::clamp(0.4 * s.density / 0.15 * 0.1, 0.02, 0.12)) << "</particle_scatter_ratio>"
         << "</particle_emitter></link></model>\n";
  }

  // ---- world
  double lat, lon;
  geoOrigin(name, lat, lon);
  std::ofstream sdf(out / "world.sdf");
  sdf << "<?xml version=\"1.0\"?>\n<!-- generated by iad_gz_sim/world_compiler (" << name << ", seed " << seed << ") -->\n"
      << "<sdf version=\"1.10\"><world name=\"" << name << "\">\n"
      << "<physics name=\"2ms\" type=\"dart\"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor>"
      << "<real_time_update_rate>500</real_time_update_rate></physics>\n"
      << "<gravity>0 0 -9.81</gravity>\n"
      << "<plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\"/>\n"
      << "<plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>\n"
      << "<plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>\n"
      << "<plugin filename=\"gz-sim-sensors-system\" name=\"gz::sim::systems::Sensors\"><render_engine>ogre2</render_engine></plugin>\n"
      << "<plugin filename=\"gz-sim-imu-system\" name=\"gz::sim::systems::Imu\"/>\n"
      << "<plugin filename=\"gz-sim-navsat-system\" name=\"gz::sim::systems::NavSat\"/>\n"
      << "<plugin filename=\"gz-sim-particle-emitter-system\" name=\"gz::sim::systems::ParticleEmitter\"/>\n"
      << "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>"
      << "<latitude_deg>" << std::setprecision(10) << lat << "</latitude_deg><longitude_deg>" << lon
      << "</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>\n"
      << "<scene><ambient>0.50 0.50 0.52 1</ambient><background>0.66 0.75 0.86 1</background><shadows>true</shadows>"
      << "<grid>false</grid><sky></sky></scene>\n"
      << "<light type=\"directional\" name=\"sun\"><cast_shadows>true</cast_shadows><pose>0 0 200 0 0 0</pose>"
      << "<diffuse>1.0 0.96 0.88 1</diffuse><specular>0.25 0.25 0.25 1</specular><intensity>1.1</intensity>"
      << "<direction>-0.45 0.35 -0.82</direction></light>\n"
      << "<light type=\"directional\" name=\"fill\"><cast_shadows>false</cast_shadows><pose>0 0 200 0 0 0</pose>"
      << "<diffuse>0.25 0.27 0.32 1</diffuse><specular>0 0 0 1</specular><direction>0.5 -0.4 -0.6</direction></light>\n"
      << terrain.str() << site.str() << "<!--BEGIN:traffic-->\n" << traffic.str() << "<!--END:traffic-->\n"
      << "<!--BEGIN:dust-->\n" << dust.str() << "<!--END:dust-->\n"
      << "</world></sdf>\n";

  // ---- extras: the unmapped obstacle the planning demo replans around (port)
  if (name == "port") {
    std::ofstream e(out / "extras" / "unmapped_container.sdf");
    const Color c{0.75f, 0.2f, 0.15f, 1.f};
    e << "<?xml version=\"1.0\"?><sdf version=\"1.10\"><model name=\"unmapped_container\"><static>true</static>"
      << poseStr(143.0, 23.5, t.height(143.0, 23.5), 0, 0, 0.05) << "<link name=\"link\">"
      << "<collision name=\"c\"><pose>0 0 1.3 0 0 0</pose><geometry><box><size>12.2 2.45 2.6</size></box></geometry></collision>"
      << "<visual name=\"v\"><pose>0 0 1.3 0 0 0</pose><laser_retro>55</laser_retro><geometry><box><size>12.2 2.45 2.6</size></box></geometry>"
      << material(c) << "</visual></link></model></sdf>\n";
  }

  // ---- maps (site survey stand-ins)
  writePcd(buildPointcloudMap(w, name == "port" ? 1.0 : 0.5, 0.6, 0.5), out / "maps" / "pointcloud_map.pcd");
  writeCostmap(buildCostmap(w, 0.5, 5.5), out / "maps");

  {
    std::ofstream mi(out / "maps" / "map_info.yaml");
    mi << std::setprecision(10) << "# Geodetic origin of the map frame (ENU) for pointcloud_map.pcd and costmap.pgm\nsite: " << name
       << "\norigin: {latitude: " << lat << ", longitude: " << lon << ", altitude: 0.0}\n";
  }
  std::ofstream y(out / "site.yaml");
  y << std::setprecision(10) << "site: " << name << "\nseed: " << seed << "\norigin: {latitude: " << lat << ", longitude: " << lon
    << ", altitude: 0.0}\n" << std::setprecision(6) << "poses:\n";
  for (const auto& p : w.poses)
    y << "  " << p.name << ": {x: " << p.x << ", y: " << p.y << ", z: " << t.height(p.x, p.y) << ", yaw: " << p.yaw << "}\n";
  y << "routes:\n";
  for (const auto& r : w.routes) {
    y << "  " << r.name << ":\n    speed: " << r.speed << "\n    loop: " << (r.loop ? "true" : "false") << "\n    points: [";
    for (size_t i = 0; i < r.pts.size(); ++i) y << (i ? ", " : "") << "[" << r.pts[i].x() << ", " << r.pts[i].y() << "]";
    y << "]\n";
  }
  std::cout << name << ": " << w.boxes.size() << " boxes, " << w.visuals.size() << " visual boxes, " << w.cylinders.size()
            << " cylinders, " << groups.size() << " merged visual groups, " << w.actors.size() << " actors, "
            << w.dust_sources.size() << " dust sources -> " << out << "\n";
  return 0;
}
