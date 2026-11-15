// Procedural, seeded worlds: an open-pit quarry and a container terminal.
#include <cmath>
#include <random>

#include "iad_sim/noise.hpp"
#include "iad_sim/world.hpp"

namespace iad::sim {
namespace {

double smoothstep(double e0, double e1, double x) {
  const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
  return t * t * (3 - 2 * t);
}
Color rgb(float r, float g, float b, float a = 1.f) { return {r, g, b, a}; }

Box mkBox(double cx, double cy, double z_bottom, double lx, double ly, double lz, double yaw, Color col,
          float refl, bool in_map = true) {
  Box b;
  b.c = Eigen::Vector3d(cx, cy, z_bottom + lz / 2);
  b.half = Eigen::Vector3d(lx / 2, ly / 2, lz / 2);
  b.yaw = yaw, b.color = col, b.refl = refl, b.in_map = in_map;
  return b;
}

// Local-frame part (actor or composite object): x forward, z up from the ground.
Box part(double x, double y, double z_bottom, double lx, double ly, double lz, Color col, float refl = 0.45f) {
  return mkBox(x, y, z_bottom, lx, ly, lz, 0.0, col, refl, false);
}

// Place a composite (parts in local frame) into the world at (x, y, yaw) on the terrain.
void placeComposite(World& w, const std::vector<Box>& parts, double x, double y, double yaw, bool in_map = true) {
  const double c = std::cos(yaw), s = std::sin(yaw), z = w.terrain.height(x, y);
  for (Box b : parts) {
    const Eigen::Vector3d l = b.c;
    b.c = Eigen::Vector3d(x + c * l.x() - s * l.y(), y + s * l.x() + c * l.y(), z + l.z());
    b.yaw = wrapAngle(yaw + b.yaw);
    b.in_map = in_map;
    b.label = uint8_t(Label::kStatic);
    w.boxes.push_back(b);
  }
}

// Centripetal Catmull-Rom (alpha = 0.5): no loops or cusps even when control points are unevenly spaced.
// Resampled to ~ds spacing.
std::vector<Eigen::Vector2d> catmullRom(const std::vector<Eigen::Vector2d>& c, double ds) {
  std::vector<Eigen::Vector2d> dense;
  auto knot = [](double t, const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return t + std::sqrt(std::max(1e-6, (b - a).norm())); };
  for (size_t i = 0; i + 1 < c.size(); ++i) {
    const Eigen::Vector2d p1 = c[i], p2 = c[i + 1];
    const Eigen::Vector2d p0 = i ? c[i - 1] : Eigen::Vector2d(2 * p1 - p2);
    const Eigen::Vector2d p3 = i + 2 < c.size() ? c[i + 2] : Eigen::Vector2d(2 * p2 - p1);
    const double t0 = 0, t1 = knot(t0, p0, p1), t2 = knot(t1, p1, p2), t3 = knot(t2, p2, p3);
    const int n = std::max(4, int((p2 - p1).norm() / (ds * 0.25)));
    for (int k = 0; k < n; ++k) {
      const double t = t1 + (t2 - t1) * k / n;
      const Eigen::Vector2d a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
      const Eigen::Vector2d a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
      const Eigen::Vector2d a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;
      const Eigen::Vector2d b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
      const Eigen::Vector2d b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
      dense.push_back((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2);
    }
  }
  dense.push_back(c.back());
  std::vector<Eigen::Vector2d> out{dense.front()};
  double acc = 0;
  for (size_t i = 1; i < dense.size(); ++i) {
    acc += (dense[i] - dense[i - 1]).norm();
    if (acc >= ds) out.push_back(dense[i]), acc = 0;
  }
  if ((out.back() - dense.back()).norm() > 0.2 * ds) out.push_back(dense.back());
  return out;
}

std::vector<double> gaussSmooth(const std::vector<double>& z, double sigma_samples) {
  const int r = int(std::ceil(3 * sigma_samples));
  std::vector<double> out(z.size());
  for (int i = 0; i < int(z.size()); ++i) {
    double s = 0, ws = 0;
    for (int k = -r; k <= r; ++k) {
      const int j = std::clamp(i + k, 0, int(z.size()) - 1);
      const double wk = std::exp(-0.5 * k * k / (sigma_samples * sigma_samples));
      s += wk * z[size_t(j)], ws += wk;
    }
    out[size_t(i)] = s / ws;
  }
  return out;
}

// Rigid dump truck (other traffic), local frame at the rear axle.
std::vector<Box> haulTruckParts(Color body) {
  const Color dark = rgb(0.12f, 0.12f, 0.13f), cab = rgb(0.25f, 0.3f, 0.35f);
  return {part(1.0, 0, 1.5, 7.6, 4.0, 2.4, body, 0.5f),   part(5.6, 0, 0.6, 1.8, 3.6, 2.8, body, 0.5f),
          part(5.4, 1.2, 3.4, 1.6, 1.4, 1.4, cab, 0.3f),  part(0.0, 1.6, 0.0, 2.6, 1.0, 2.2, dark, 0.1f),
          part(0.0, -1.6, 0.0, 2.6, 1.0, 2.2, dark, 0.1f), part(4.3, 1.6, 0.0, 2.2, 0.9, 2.0, dark, 0.1f),
          part(4.3, -1.6, 0.0, 2.2, 0.9, 2.0, dark, 0.1f)};
}

std::vector<Box> pickupParts() {
  const Color white = rgb(0.92f, 0.92f, 0.9f), glass = rgb(0.2f, 0.25f, 0.3f), dark = rgb(0.1f, 0.1f, 0.1f);
  return {part(1.3, 0, 0.45, 5.2, 1.9, 0.8, white, 0.6f), part(1.6, 0, 1.25, 2.2, 1.8, 0.7, glass, 0.2f),
          part(1.3, 0, 0.0, 4.0, 1.7, 0.45, dark, 0.1f), part(3.0, 0, 2.05, 0.4, 1.2, 0.25, rgb(1.f, 0.55f, 0.f), 0.9f)};
}

std::vector<Box> workerParts() {
  return {part(0, 0, 0.0, 0.35, 0.5, 0.85, rgb(0.15f, 0.2f, 0.35f), 0.3f),
          part(0, 0, 0.85, 0.35, 0.55, 0.65, rgb(1.0f, 0.55f, 0.05f), 0.9f),
          part(0, 0, 1.5, 0.26, 0.26, 0.28, rgb(0.95f, 0.85f, 0.1f), 0.6f)};
}

std::vector<Box> excavatorParts() {
  const Color y = rgb(0.95f, 0.68f, 0.08f), d = rgb(0.13f, 0.13f, 0.13f);
  std::vector<Box> p{part(0, 1.6, 0, 5.6, 1.0, 1.2, d, 0.1f),     part(0, -1.6, 0, 5.6, 1.0, 1.2, d, 0.1f),
                     part(-0.6, 0, 1.2, 5.0, 3.4, 1.9, y, 0.5f),  part(1.2, 1.0, 3.1, 1.6, 1.2, 1.6, rgb(0.3f, 0.35f, 0.4f), 0.3f),
                     part(-2.6, 0, 1.2, 1.2, 3.2, 1.6, d, 0.2f)};
  for (int k = 0; k < 5; ++k) p.push_back(part(2.4 + 1.1 * k, -0.4, 3.0 + 0.7 * k - (k > 3 ? 1.6 : 0), 1.3, 0.7, 0.8, y));
  for (int k = 0; k < 3; ++k) p.push_back(part(7.6 + 0.6 * k, -0.4, 4.2 - 1.3 * k, 0.8, 0.6, 1.4, y));
  p.push_back(part(8.6, -0.4, 0.3, 1.5, 1.7, 1.2, d, 0.2f));  // bucket
  return p;
}

std::vector<Box> dozerParts() {
  const Color y = rgb(0.95f, 0.68f, 0.08f), d = rgb(0.13f, 0.13f, 0.13f);
  return {part(0, 1.4, 0, 4.6, 0.9, 1.1, d, 0.1f), part(0, -1.4, 0, 4.6, 0.9, 1.1, d, 0.1f),
          part(0, 0, 1.1, 4.0, 2.4, 1.4, y),       part(-0.6, 0, 2.5, 1.8, 2.0, 1.4, rgb(0.3f, 0.35f, 0.4f), 0.3f),
          part(3.0, 0, 0, 0.5, 4.2, 1.6, y)};
}

std::vector<Box> straddleCarrierParts(bool loaded, Color box_col) {
  const Color c = rgb(0.92f, 0.62f, 0.12f), d = rgb(0.15f, 0.15f, 0.16f);
  std::vector<Box> p;
  for (double x : {-4.5, 4.5})
    for (double y : {-2.6, 2.6}) p.push_back(part(x, y, 0.0, 0.8, 0.7, 12.0, c));
  p.push_back(part(0, 2.6, 12.0, 10.0, 0.8, 1.4, c));
  p.push_back(part(0, -2.6, 12.0, 10.0, 0.8, 1.4, c));
  p.push_back(part(4.0, 0, 12.0, 1.4, 5.0, 1.2, c));
  p.push_back(part(4.6, 2.6, 13.4, 1.8, 1.4, 1.8, rgb(0.3f, 0.35f, 0.4f), 0.3f));
  for (double x : {-4.5, 4.5}) p.push_back(part(x, 0, 0.0, 1.6, 6.0, 0.6, d, 0.1f));
  if (loaded) p.push_back(part(0, 0, 3.5, 12.0, 2.4, 2.6, box_col, 0.5f));
  return p;
}

std::vector<Box> terminalTruckParts(Color box_col) {
  const Color cab = rgb(0.85f, 0.85f, 0.85f), d = rgb(0.12f, 0.12f, 0.12f);
  return {part(9.0, 0, 0.4, 3.0, 2.5, 2.6, cab, 0.55f), part(2.0, 0, 0.6, 13.0, 2.4, 0.6, d, 0.2f),
          part(2.0, 0, 1.2, 12.2, 2.44, 2.6, box_col, 0.5f), part(9.0, 0, 0.0, 2.4, 2.4, 0.5, d, 0.1f),
          part(-3.0, 0, 0.0, 3.0, 2.4, 0.6, d, 0.1f)};
}

const std::vector<Color>& containerColors() {
  static const std::vector<Color> k = {rgb(0.13f, 0.33f, 0.62f), rgb(0.70f, 0.17f, 0.14f), rgb(0.20f, 0.47f, 0.30f),
                                       rgb(0.86f, 0.46f, 0.12f), rgb(0.55f, 0.57f, 0.58f), rgb(0.86f, 0.86f, 0.83f),
                                       rgb(0.45f, 0.28f, 0.18f), rgb(0.15f, 0.50f, 0.52f), rgb(0.86f, 0.71f, 0.15f),
                                       rgb(0.40f, 0.18f, 0.40f)};
  return k;
}

}  // namespace

// =====================================================================================  quarry
World makeQuarry(uint32_t seed) {
  World w;
  w.name = "quarry";
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  const double X0 = -200, Y0 = -150, res = 0.5;
  const int nx = int(400 / res) + 1, ny = int(300 / res) + 1;
  w.terrain.init(nx, ny, res, X0, Y0);

  const Eigen::Vector2d pit(-120, 22);
  const double pit_rb = 36, pit_rr = 70, pit_depth = 8;
  auto natural = [&](double x, double y) {
    double z = 3.6 * fbm(x / 150, y / 150, seed) + 1.1 * fbm(x / 40, y / 40, seed + 7) +
               0.35 * fbm(x / 8, y / 8, seed + 13) + 0.07 * fbm(x / 1.7, y / 1.7, seed + 19);
    const double r = (Eigen::Vector2d(x, y) - pit).norm();
    if (r < pit_rr) {  // benched pit: steep faces, flat benches
      const double u = r < pit_rb ? 0.0 : (r - pit_rb) / (pit_rr - pit_rb);
      const double steps = 3.0, fu = u * steps, fl = std::floor(fu);
      const double bench = (fl + smoothstep(0.0, 0.35, fu - fl)) / steps;
      const double depth = pit_depth * (1.0 - std::min(1.0, bench));
      z = z * (0.25 + 0.75 * u) - depth;
    }
    z += 2.0 * smoothstep(60, 140, x);                    // dump plateau
    z -= 9.0 * smoothstep(181, 188, x) * (std::abs(y - 5) < 70 ? 1.0 : smoothstep(95, 70, std::abs(y - 5)));
    static const double piles[3][4] = {{110, -95, 18, 9}, {62, -110, 14, 7}, {150, -70, 11, 5}};
    for (const auto& pl : piles) {
      const double cx = pl[0], cy = pl[1], cr = pl[2], ch = pl[3];
      const double rr = std::hypot(x - cx, y - cy);
      if (rr < cr) z += ch * (1.0 - rr / cr) * (1.0 - rr / cr) + 0.25 * fbm(x / 2, y / 2, seed + 3);
    }
    return z;
  };

  // ---- haul road: spline centreline, grade-limited design height, berms, cut/fill blend
  const std::vector<Eigen::Vector2d> ctrl = {{-138, 24}, {-112, 14}, {-80, 2},  {-46, -14}, {-10, -26}, {28, -18},
                                             {62, 4},    {96, 22},   {128, 14}, {154, 4},   {170, 2}};
  const auto road = catmullRom(ctrl, 1.0);
  std::vector<double> zr(road.size());
  for (size_t i = 0; i < road.size(); ++i) zr[i] = natural(road[i].x(), road[i].y());
  std::vector<double> zd = gaussSmooth(zr, 8.0);
  const double gmax = 0.085;
  for (int pass = 0; pass < 3; ++pass) {
    for (size_t i = 1; i < zd.size(); ++i) zd[i] = std::clamp(zd[i], zd[i - 1] - gmax, zd[i - 1] + gmax);
    for (size_t i = zd.size() - 1; i-- > 0;) zd[i] = std::clamp(zd[i], zd[i + 1] - gmax, zd[i + 1] + gmax);
  }
  zd = gaussSmooth(zd, 3.0);
  const Eigen::Vector2d mud_c(62, 4), load_c = road.front(), dump_c(166, 3);
  const double road_half = 9.0, berm_w = 4.0, berm_h = 1.4, blend = 9.0, pad_r = 26.0;
  auto nearPad = [&](const Eigen::Vector2d& p) { return (p - load_c).norm() < pad_r || (p - dump_c).norm() < pad_r; };

  std::vector<float> best_d(size_t(nx) * ny, 1e9f), best_z(size_t(nx) * ny, 0.f);
  std::vector<uint8_t> best_berm(size_t(nx) * ny, 0);
  const double R = road_half + berm_w + blend;
  for (size_t k = 0; k < road.size(); ++k) {
    const int i0 = int((road[k].x() - R - X0) / res), i1 = int((road[k].x() + R - X0) / res) + 1;
    const int j0 = int((road[k].y() - R - Y0) / res), j1 = int((road[k].y() + R - Y0) / res) + 1;
    for (int j = std::max(0, j0); j <= std::min(ny - 1, j1); ++j)
      for (int i = std::max(0, i0); i <= std::min(nx - 1, i1); ++i) {
        const double d = std::hypot(X0 + i * res - road[k].x(), Y0 + j * res - road[k].y());
        const size_t c = size_t(j) * nx + i;
        if (d < best_d[c]) best_d[c] = float(d), best_z[c] = float(zd[k]), best_berm[c] = !nearPad(road[k]);
      }
  }
  const double load_z = zd.front(), dump_z = zd.back();
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      const double x = X0 + i * res, y = Y0 + j * res;
      double z = natural(x, y);
      uint8_t m = kDirt;
      const double rough = fbm(x / 9, y / 9, seed + 31);
      if (rough > 0.55) m = kRock;
      const size_t c = size_t(j) * nx + i;
      const double d = best_d[c], zdes = best_z[c];
      const Eigen::Vector2d p(x, y);
      // pads: flat, open (no berm)
      for (int pad = 0; pad < 2; ++pad) {
        const Eigen::Vector2d pc = pad ? dump_c : load_c;
        const double pz = pad ? dump_z : load_z;
        const double rp = (p - pc).norm();
        if (rp < pad_r + blend) {
          const double t = smoothstep(pad_r, pad_r + blend, rp);
          z = pz + 0.04 * fbm(x / 3, y / 3, seed + 5) + t * (z - pz);
          if (rp < pad_r) m = kDirt;
        }
      }
      if (d < R) {
        const double road_z = zdes + 0.035 * fbm(x / 2.5, y / 2.5, seed + 41);
        if (d <= road_half) {
          z = road_z, m = kGravel;
        } else if (d <= road_half + berm_w) {
          const double u = (d - road_half) / berm_w;
          z = zdes + (best_berm[c] ? berm_h * std::sin(kPi * u) : 0.0), m = kDirt;
        } else {
          const double t = smoothstep(0.0, 1.0, (d - road_half - berm_w) / blend);
          z = zdes + t * (z - zdes);
        }
      }
      const double rm = (p - mud_c).norm();
      if (rm < 15 && d < road_half + 2) {
        z -= 0.35 * (1 - sq(rm / 15));
        if (rm < 12) m = kMud;
      }
      // safety berm at the tipping edge
      if (x > 174 && x < 178.5 && std::abs(y - 4) < 34) z = std::max(z, dump_z + 1.6 * std::sin(kPi * (x - 174) / 4.5));
      w.terrain.H(i, j) = float(z);
      w.terrain.M(i, j) = m;
    }
  w.terrain.updateBounds();

  // ---- static objects
  auto onRoad = [&](double x, double y) {
    const int i = std::clamp(int((x - X0) / res), 0, nx - 1), j = std::clamp(int((y - Y0) / res), 0, ny - 1);
    return best_d[size_t(j) * nx + i] < road_half + berm_w + 3 || nearPad({x, y});
  };
  const Color rock = rgb(0.47f, 0.44f, 0.41f);
  for (int k = 0, tries = 0; k < 90 && tries < 5000; ++tries) {
    const double x = X0 + 10 + U(rng) * 380, y = Y0 + 10 + U(rng) * 280;
    if (onRoad(x, y) || x > 178) continue;
    Cylinder c;
    c.x = x, c.y = y, c.r = 0.4 + 1.1 * U(rng);
    c.z0 = w.terrain.height(x, y) - 0.3, c.z1 = c.z0 + 0.3 + c.r * (0.8 + 0.8 * U(rng));
    c.color = rock, c.refl = 0.35f, c.rock = true;
    w.cylinders.push_back(c), ++k;
  }
  placeComposite(w, excavatorParts(), -156, 22, 0.15);
  placeComposite(w, dozerParts(), 152, 44, -2.2);
  placeComposite(w, haulTruckParts(rgb(0.9f, 0.62f, 0.1f)), -40, 44, 2.6);  // parked
  // Crusher + an overhead conveyor that crosses the haul road (overhang: not an obstacle).
  const Color steel = rgb(0.55f, 0.58f, 0.62f), rust = rgb(0.62f, 0.38f, 0.22f);
  {
    const double gx = 128, gy = 70, gz = w.terrain.height(gx, gy);
    w.boxes.push_back(mkBox(gx, gy, gz, 16, 12, 14, 0.1, steel, 0.5f));
    w.boxes.push_back(mkBox(gx - 2, gy - 10, gz + 6, 7, 7, 6, 0.1, rust, 0.4f));
    const Eigen::Vector2d A(121, 58), B(94, -26);
    const double yaw = std::atan2(B.y() - A.y(), B.x() - A.x()), L = (B - A).norm();
    const int n = 9;
    for (int k = 0; k < n; ++k) {
      const double u = (k + 0.5) / n;
      const Eigen::Vector2d c = A + u * (B - A);
      const double zb = std::max(gz + 12 - 7 * u, w.terrain.height(c.x(), c.y()) + 7.5);
      w.boxes.push_back(mkBox(c.x(), c.y(), zb, L / n + 0.2, 2.6, 1.6, yaw, rust, 0.45f));
      if (!onRoad(c.x(), c.y()))
        w.cylinders.push_back({c.x(), c.y(), w.terrain.height(c.x(), c.y()) - 0.2, zb, 0.35, 0.45f, steel,
                               uint8_t(Label::kStatic), true});
    }
  }
  for (int k = 0; k < 3; ++k) {  // site office cabins
    const double x = 18 + k * 7.5, y = 72;
    w.boxes.push_back(mkBox(x, y, w.terrain.height(x, y), 6.1, 2.6, 2.7, 0, rgb(0.85f, 0.85f, 0.8f), 0.55f));
  }
  {
    const double x = 40, y = 70, z = w.terrain.height(x, y);
    w.cylinders.push_back({x, y, z, z + 3.5, 1.6, 0.5f, rgb(0.8f, 0.8f, 0.78f), uint8_t(Label::kStatic), true});
  }
  static const double towers[3][2] = {{-100, 40}, {0, -6}, {150, 30}};
  for (const auto& tw : towers) {  // light towers
    const double x = tw[0], y = tw[1], z = w.terrain.height(x, y);
    w.cylinders.push_back({x, y, z, z + 9, 0.2, 0.5f, steel, uint8_t(Label::kStatic), true});
    w.boxes.push_back(mkBox(x, y, z, 2.2, 1.6, 1.2, 0.3, rgb(0.9f, 0.9f, 0.9f), 0.5f));
    w.boxes.push_back(mkBox(x, y, z + 9, 1.8, 0.4, 1.0, 0.3, rgb(0.95f, 0.95f, 0.8f), 0.9f));
  }
  // Unmapped obstacles on the haul road (the site map does not know them; perception must).
  auto roadPoint = [&](double frac, double lateral) {
    const size_t k = std::min(road.size() - 2, size_t(frac * double(road.size())));
    const Eigen::Vector2d t = (road[k + 1] - road[k]).normalized(), nrm(-t.y(), t.x());
    return std::pair<Eigen::Vector2d, double>{Eigen::Vector2d(road[k] + lateral * nrm), std::atan2(t.y(), t.x())};
  };
  {
    auto [p, yaw] = roadPoint(0.40, 3.6);
    Cylinder c;
    c.x = p.x(), c.y = p.y(), c.r = 1.0, c.z0 = w.terrain.height(p.x(), p.y()) - 0.2, c.z1 = c.z0 + 1.6;
    c.color = rgb(0.55f, 0.5f, 0.45f), c.refl = 0.35f, c.in_map = false, c.rock = true;
    w.cylinders.push_back(c);
    auto [q, yaw2] = roadPoint(0.72, -5.5);
    placeComposite(w, pickupParts(), q.x(), q.y(), yaw2 + 0.25, false);
  }

  // ---- dynamic actors
  {
    Actor truck;
    truck.kind = "haul_truck";
    std::vector<Eigen::Vector2d> loop;
    for (size_t k = 0; k < road.size(); k += 3) {
      const size_t k2 = std::min(k + 1, road.size() - 1), k1 = k2 - 1;
      const Eigen::Vector2d t = (road[k2] - road[k1]).normalized(), nrm(-t.y(), t.x());
      loop.push_back(road[k] - 4.0 * nrm);
    }
    for (size_t k = road.size(); k-- > 0;) {
      if (k % 3) continue;
      const size_t k2 = std::min(k + 1, road.size() - 1), k1 = k2 - 1;
      const Eigen::Vector2d t = (road[k2] - road[k1]).normalized(), nrm(-t.y(), t.x());
      loop.push_back(road[k] + 4.0 * nrm);
    }
    loop.push_back(loop.front());
    truck.path = loop, truck.loop = true, truck.speed = 6.5, truck.emits_dust = true;
    double L = 0;
    for (size_t i = 0; i + 1 < loop.size(); ++i) L += (loop[i + 1] - loop[i]).norm();
    truck.s = 0.62 * L;
    truck.parts = haulTruckParts(rgb(0.85f, 0.55f, 0.1f));
    w.actors.push_back(truck);
  }
  {
    Actor lv;
    lv.kind = "light_vehicle";
    lv.path = {{40, -70}, {40, 40}, {48, 52}, {60, 52}, {68, 40}, {68, -70}, {60, -80}, {48, -80}, {40, -70}};
    lv.loop = true, lv.speed = 6.0, lv.emits_dust = true, lv.s = 20;
    lv.parts = pickupParts();
    w.actors.push_back(lv);
  }
  {
    Actor wk;
    wk.kind = "worker";
    wk.path = {{-126, 44}, {-112, 40}};
    wk.loop = false, wk.speed = 1.1, wk.wait_at_end = 3.0;
    wk.parts = workerParts();
    w.actors.push_back(wk);
  }

  // ---- dust: loading face, tipping edge, crusher, and a wind-blown stretch of haul road
  auto src = [&](double x, double y, double rate, double r, double dens, double spread) {
    DustSource s;
    s.c = Eigen::Vector3d(x, y, w.terrain.height(x, y) + 2.0), s.rate = rate, s.radius = r, s.density = dens, s.spread = spread;
    w.dust_sources.push_back(s);
  };
  src(-150, 18, 0.8, 4.0, 0.25, 5.0);
  src(184, 4, 0.5, 4.5, 0.20, 10.0);  // tipping edge: over the drop, beyond the safety berm
  src(124, 58, 0.5, 5.0, 0.15, 6.0);
  {
    auto [p, yaw] = roadPoint(0.30, 0.0);
    src(p.x(), p.y(), 1.6, 6.0, 0.13, 13.0);
  }
  w.wind = Eigen::Vector2d(1.3, 0.5);

  // ---- routes + named poses
  Route haul;
  haul.name = "haul", haul.speed = 7.0;
  for (size_t k = 8; k + 6 < road.size(); ++k) haul.pts.push_back(road[k]);
  w.routes.push_back(haul);
  Route back = haul;
  back.name = "haul_back";
  std::reverse(back.pts.begin(), back.pts.end());
  w.routes.push_back(back);
  auto yawAt = [&](size_t k) { return std::atan2(road[k + 1].y() - road[k].y(), road[k + 1].x() - road[k].x()); };
  w.poses.push_back({"start", road[8].x(), road[8].y(), yawAt(8)});
  w.poses.push_back({"load", -137.0, 17.0, 0.35});
  w.poses.push_back({"dump", 167.5, 3.0, kPi});
  w.poses.push_back({"haul_mid", road[road.size() / 2].x(), road[road.size() / 2].y(), yawAt(road.size() / 2)});
  const size_t k75 = road.size() * 3 / 4;
  w.poses.push_back({"dump_approach", road[k75].x(), road[k75].y(), yawAt(k75)});
  return w;
}

// =====================================================================================  port
World makePort(uint32_t seed) {
  World w;
  w.name = "port";
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  const double X0 = -60, Y0 = -80, res = 0.5;
  const int nx = int(520 / res) + 1, ny = int(360 / res) + 1;
  w.terrain.init(nx, ny, res, X0, Y0);
  const double quay_y = 250;
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      const double x = X0 + i * res, y = Y0 + j * res;
      double z = 0.05 * fbm(x / 60, y / 60, seed) + 0.01 * fbm(x / 3, y / 3, seed + 3);
      uint8_t m = (y > 192 && y < quay_y) ? kConcrete : kAsphalt;
      if (y > quay_y + 1.5) z = -2.5, m = kWater;
      else if (y > quay_y) z = -2.5 * smoothstep(quay_y, quay_y + 1.5, y);
      w.terrain.H(i, j) = float(z);
      w.terrain.M(i, j) = m;
    }
  w.terrain.updateBounds();

  // ---- container blocks (one box per stack for ray casting; tiers are drawn individually)
  const auto& cols = containerColors();
  const double bay = 12.5, row = 2.9, cl = 12.19, cw = 2.44, ch = 2.59;
  const std::vector<double> bx = {0, 150, 300}, by = {10, 60, 110, 160};
  std::discrete_distribution<int> tiers_d({4, 10, 18, 25, 26, 17});
  for (double x0 : bx)
    for (double y0 : by)
      for (int b = 0; b < 10; ++b)
        for (int r = 0; r < 6; ++r) {
          int tiers = tiers_d(rng);
          if (x0 == 150 && b < 6) tiers = std::max(tiers, 4);  // tall canyon along the x=137.5 aisle
          if (tiers == 0) continue;
          const double cx = x0 + bay * (b + 0.5), cy = y0 + row * (r + 0.5);
          Box stack = mkBox(cx, cy, 0.0, cl, cw, ch * tiers, 0.0, cols[0], 0.5f);
          stack.visible = false;
          w.boxes.push_back(stack);
          for (int t = 0; t < tiers; ++t) {
            Box vis = mkBox(cx, cy, ch * t + 0.02, cl, cw, ch - 0.04, 0.0, cols[size_t(U(rng) * cols.size()) % cols.size()], 0.5f);
            w.visuals.push_back(vis);  // visual tiers
          }
        }
  // ---- RTGs (one per block) spanning the block + its truck lane
  const Color rtg = rgb(0.88f, 0.88f, 0.86f), dark = rgb(0.2f, 0.2f, 0.22f);
  for (double x0 : bx)
    for (double y0 : by) {
      const double xr = x0 + bay * (2 + int(U(rng) * 6)), ys = y0 - 7.5, yn = y0 + 19.0, top = 19.5;
      for (double xx : {xr - 4.8, xr + 4.8}) {
        for (double yy : {ys, yn}) w.boxes.push_back(mkBox(xx, yy, 0, 1.0, 1.4, top, 0, rtg, 0.55f));
        w.boxes.push_back(mkBox(xx, (ys + yn) / 2, top, 1.4, yn - ys + 1.4, 1.8, 0, rtg, 0.55f));
        w.boxes.push_back(mkBox(xx, ys, 0, 1.6, 3.2, 1.2, 0, dark, 0.2f));
        w.boxes.push_back(mkBox(xx, yn, 0, 1.6, 3.2, 1.2, 0, dark, 0.2f));
      }
      w.boxes.push_back(mkBox(xr, ys, top - 1.5, 10.6, 1.2, 1.4, 0, rtg, 0.55f));
      w.boxes.push_back(mkBox(xr, yn, top - 1.5, 10.6, 1.2, 1.4, 0, rtg, 0.55f));
      w.boxes.push_back(mkBox(xr + 2.0, yn - 2.0, top + 1.8, 3.0, 3.0, 2.2, 0, rgb(0.15f, 0.35f, 0.6f), 0.4f));
    }
  // ---- ship-to-shore cranes over the quay lanes (GNSS-denied zone)
  const Color sts = rgb(0.78f, 0.22f, 0.16f), white = rgb(0.9f, 0.9f, 0.9f);
  const double y_land = 205, y_sea = 242;
  for (double xc : {90.0, 200.0, 310.0}) {
    for (double xx : {xc - 13, xc + 13}) {
      for (double yy : {y_land, y_sea}) w.boxes.push_back(mkBox(xx, yy, 0, 2.0, 2.0, 42, 0, sts, 0.5f));
      w.boxes.push_back(mkBox(xx, (y_land + y_sea) / 2, 40, 2.4, y_sea - y_land + 2, 3.0, 0, sts, 0.5f));
      w.boxes.push_back(mkBox(xx, (y_land + y_sea) / 2, 18, 1.0, y_sea - y_land, 1.0, 0, sts, 0.5f));
    }
    // Gantry: the sill beams and bogies run along the quay rails (x); trucks drive between the legs.
    for (double yy : {y_land, y_sea}) {
      w.boxes.push_back(mkBox(xc, yy, 39, 28, 2.4, 3.0, 0, sts, 0.5f));
      w.boxes.push_back(mkBox(xc, yy, 0, 30, 1.6, 2.0, 0, dark, 0.2f));
    }
    w.boxes.push_back(mkBox(xc, 235, 45, 7.0, 150.0, 3.5, 0, white, 0.6f));  // boom over the water
    w.boxes.push_back(mkBox(xc, 196, 44, 12.0, 12.0, 7.0, 0, white, 0.6f));   // machinery house
    w.boxes.push_back(mkBox(xc - 2, 230, 33, 3.0, 4.0, 3.0, 0, rgb(0.2f, 0.35f, 0.55f), 0.4f));  // trolley/spreader
    w.boxes.push_back(mkBox(xc - 2, 230, 9, 12.4, 2.6, 0.6, 0, dark, 0.2f));
  }
  for (double x = -40; x < 450; x += 22)  // bollards
    w.cylinders.push_back({x, quay_y - 1.0, 0, 0.8, 0.35, 0.3f, rgb(0.85f, 0.8f, 0.2f), uint8_t(Label::kStatic), true});
  // ---- light poles at aisle intersections
  for (double x : {-25.0, 137.5, 287.5, 437.5})
    for (double y : {-15.0, 47.0, 97.0, 147.0, 190.0}) {
      w.cylinders.push_back({x + 6, y + 6, 0, 30, 0.35, 0.5f, rgb(0.6f, 0.62f, 0.65f), uint8_t(Label::kStatic), true});
      w.boxes.push_back(mkBox(x + 6, y + 6, 30, 3.0, 3.0, 0.8, 0, rgb(0.95f, 0.95f, 0.85f), 0.8f));
      w.boxes.push_back(mkBox(x + 6, y + 6, 0, 1.4, 1.4, 1.0, 0, rgb(0.85f, 0.75f, 0.2f), 0.6f));
    }
  // ---- buildings + parked trailers
  w.boxes.push_back(mkBox(200, -62, 0, 60, 28, 12, 0, rgb(0.7f, 0.72f, 0.75f), 0.5f));
  w.boxes.push_back(mkBox(40, -58, 0, 32, 16, 9, 0, rgb(0.85f, 0.83f, 0.78f), 0.5f));
  w.boxes.push_back(mkBox(330, -60, 0, 40, 20, 7, 0, rgb(0.6f, 0.65f, 0.6f), 0.5f));
  for (int k = 0; k < 8; ++k)
    w.boxes.push_back(mkBox(330 + k * 12.0, -32, 0, 2.5, 12.5, 1.6, 0, rgb(0.25f, 0.25f, 0.28f), 0.3f));
  // Unmapped: a container set down in the canyon aisle, and pallets on the quay.
  w.boxes.push_back(mkBox(141.5, 92, 0, 2.44, 12.19, 2.59, 0, cols[3], 0.5f, false));
  w.boxes.push_back(mkBox(252, 214, 0, 2.4, 2.4, 1.6, 0.3, rgb(0.6f, 0.45f, 0.25f), 0.35f, false));

  // lane paint (visual)
  for (double y : {-15.0, 222.0})
    for (double x = -40; x < 450; x += 9) w.visuals.push_back(mkBox(x, y, 0.02, 4.5, 0.18, 0.02, 0, rgb(0.92f, 0.92f, 0.9f), 0.6f));
  for (double x : {137.5, 287.5, -25.0, 437.5})
    for (double y = -10; y < 190; y += 9) w.visuals.push_back(mkBox(x, y, 0.02, 0.18, 4.5, 0.02, 0, rgb(0.95f, 0.85f, 0.2f), 0.6f));

  // ---- dynamic actors (kept off the AGV routes; straddle carriers and terminal tractors shuttle back and forth)
  {
    Actor sc;
    sc.kind = "straddle_carrier";
    sc.path = {{10, 97}, {118, 97}};
    sc.loop = false, sc.bidirectional = true, sc.speed = 4.0, sc.s = 40, sc.wait_at_end = 3.0;
    sc.parts = straddleCarrierParts(true, cols[1]);
    w.actors.push_back(sc);
    Actor sc2 = sc;
    sc2.path = {{305, 147}, {425, 147}};
    sc2.s = 60, sc2.parts = straddleCarrierParts(false, cols[0]);
    w.actors.push_back(sc2);
  }
  {
    Actor tt;
    tt.kind = "terminal_truck";
    tt.path = {{-25, 233}, {425, 233}};
    tt.loop = false, tt.bidirectional = true, tt.speed = 6.0, tt.s = 150, tt.wait_at_end = 2.0;
    tt.parts = terminalTruckParts(cols[0]);
    w.actors.push_back(tt);
    Actor tt2 = tt;
    tt2.s = 380, tt2.dir = -1, tt2.parts = terminalTruckParts(cols[7]);
    w.actors.push_back(tt2);
  }
  {
    Actor wk;
    wk.kind = "worker";
    wk.path = {{196, 238}, {196, 246}};
    wk.loop = false, wk.speed = 1.0, wk.wait_at_end = 4.0;
    wk.parts = workerParts();
    w.actors.push_back(wk);
  }

  // ---- routes + named poses
  // Localization loop: open sky -> stack canyon -> the truck lane under an RTG (13 m stacks 3 m away)
  // -> cross aisle -> quay lane under three ship-to-shore cranes -> back into the canyon.
  Route loc;
  loc.name = "loc_loop", loc.speed = 7.0;
  loc.pts = catmullRom({{55, -15}, {110, -15}, {130, -8}, {137.5, 8}, {137.5, 38}, {142, 52}, {155, 57}, {270, 57},
                        {282, 64}, {287.5, 80}, {287.5, 180}, {282, 205}, {262, 220}, {200, 220}, {150, 220},
                        {141, 210}, {137.5, 190}, {137.5, 160}},
                       1.0);
  w.routes.push_back(loc);
  Route cal;
  cal.name = "calib_route", cal.speed = 3.0;
  // Calibration drive: a straight run past parallel walls, then a 90 degree turn into the stack aisle
  // whose block end faces are normal to x (without them the longitudinal offset is unobservable).
  cal.pts = catmullRom({{-10, -12}, {40, -12}, {90, -14}, {118, -12}, {133, -2}, {137.5, 15}, {137.5, 40}}, 1.0);
  w.routes.push_back(cal);
  w.poses.push_back({"start", loc.pts.front().x(), loc.pts.front().y(), 0.0});
  w.poses.push_back({"calib_start", -10, -12, -0.05});
  w.poses.push_back({"crane2_lane", 215.0, 222.0, kPi});
  w.poses.push_back({"rtg_lane", 232.0, 56.0, 0.0});
  w.poses.push_back({"rtg_lane_end", 262.0, 57.0, 0.0});
  w.poses.push_back({"gate", -20.0, -15.0, 0.0});
  return w;
}

}  // namespace iad::sim
