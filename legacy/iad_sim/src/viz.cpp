#include "iad_sim/viz.hpp"

#include "iad_sim/noise.hpp"

namespace iad::sim {
namespace {
const Eigen::Vector3d kSun = Eigen::Vector3d(0.45, -0.35, 0.82).normalized();

std_msgs::msg::ColorRGBA toRGBA(const Color& c) {
  std_msgs::msg::ColorRGBA o;
  o.r = c[0], o.g = c[1], o.b = c[2], o.a = c[3];
  return o;
}
geometry_msgs::msg::Point toPt(const Eigen::Vector3d& p) {
  geometry_msgs::msg::Point o;
  o.x = p.x(), o.y = p.y(), o.z = p.z();
  return o;
}
Color mul(const Color& c, double k) { return {float(c[0] * k), float(c[1] * k), float(c[2] * k), c[3]}; }
}  // namespace

Color shade(const Color& c, const Eigen::Vector3d& n, double ambient) {
  const double k = ambient + (1.0 - ambient) * std::max(0.0, n.dot(kSun));
  return {float(std::min(1.0, c[0] * k)), float(std::min(1.0, c[1] * k)), float(std::min(1.0, c[2] * k)), c[3]};
}

MeshBuilder::MeshBuilder(const std::string& frame, const std::string& ns, int id) {
  m_.header.frame_id = frame;
  m_.ns = ns, m_.id = id;
  m_.type = Marker::TRIANGLE_LIST, m_.action = Marker::ADD;
  m_.pose.orientation.w = 1.0;
  m_.scale.x = m_.scale.y = m_.scale.z = 1.0;
  m_.color.r = m_.color.g = m_.color.b = m_.color.a = 1.0;
}

void MeshBuilder::tri(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Color& col) {
  const Eigen::Vector3d n = (b - a).cross(c - a).normalized();
  const auto s = toRGBA(shade(col, n));
  for (const auto* p : {&a, &b, &c}) m_.points.push_back(toPt(*p)), m_.colors.push_back(s);
}

void MeshBuilder::tri(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Color& ca,
                      const Color& cb, const Color& cc) {
  m_.points.push_back(toPt(a)), m_.points.push_back(toPt(b)), m_.points.push_back(toPt(c));
  m_.colors.push_back(toRGBA(ca)), m_.colors.push_back(toRGBA(cb)), m_.colors.push_back(toRGBA(cc));
}

void MeshBuilder::quad(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                       const Eigen::Vector3d& d, const Color& col) {
  tri(a, b, c, col), tri(a, c, d, col);
}

void MeshBuilder::box(const Box& bx) {
  const double c = std::cos(bx.yaw), s = std::sin(bx.yaw);
  auto P = [&](double sx, double sy, double sz) {
    const double lx = sx * bx.half.x(), ly = sy * bx.half.y();
    return Eigen::Vector3d(bx.c.x() + c * lx - s * ly, bx.c.y() + s * lx + c * ly, bx.c.z() + sz * bx.half.z());
  };
  const Color top = mul(bx.color, 1.08);
  quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), top);       // +z
  quad(P(-1, -1, -1), P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), bx.color);  // -z
  quad(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), bx.color);   // +x
  quad(P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), bx.color);  // -x
  quad(P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), bx.color);   // +y
  quad(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), bx.color);  // -y
}

void MeshBuilder::cylinder(double x, double y, double z0, double z1, double r, const Color& col, int sides) {
  for (int k = 0; k < sides; ++k) {
    const double a0 = 2 * kPi * k / sides, a1 = 2 * kPi * (k + 1) / sides;
    const Eigen::Vector3d p0(x + r * std::cos(a0), y + r * std::sin(a0), z0), p1(x + r * std::cos(a1), y + r * std::sin(a1), z0);
    const Eigen::Vector3d q0(p0.x(), p0.y(), z1), q1(p1.x(), p1.y(), z1);
    quad(p0, p1, q1, q0, col);
    tri(Eigen::Vector3d(x, y, z1), q0, q1, mul(col, 1.08));
  }
}

void MeshBuilder::wheel(const Eigen::Vector3d& c, double r, double width, const Color& tire, const Color& hub) {
  const int sides = 16;
  const double y0 = c.y() - width / 2, y1 = c.y() + width / 2;
  for (int k = 0; k < sides; ++k) {
    const double a0 = 2 * kPi * k / sides, a1 = 2 * kPi * (k + 1) / sides;
    const Eigen::Vector3d p0(c.x() + r * std::cos(a0), y0, c.z() + r * std::sin(a0));
    const Eigen::Vector3d p1(c.x() + r * std::cos(a1), y0, c.z() + r * std::sin(a1));
    const Eigen::Vector3d q0(p0.x(), y1, p0.z()), q1(p1.x(), y1, p1.z());
    quad(p0, q0, q1, p1, tire);
    tri(Eigen::Vector3d(c.x(), y1, c.z()), q1, q0, hub);
    tri(Eigen::Vector3d(c.x(), y0, c.z()), p0, p1, hub);
  }
}

void MeshBuilder::rock(double x, double y, double z0, double z1, double r, const Color& col, uint32_t seed) {
  // Perturbed icosahedron, flattened underneath: a low-poly boulder.
  const double t = 1.618033988749895;
  std::vector<Eigen::Vector3d> v = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                                    {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
  static const int f[20][3] = {{0, 11, 5}, {0, 5, 1},  {0, 1, 7},  {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                               {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
                               {3, 8, 9},  {4, 9, 5},  {2, 4, 11}, {6, 2, 10}, {8, 6, 7},   {9, 8, 1}};
  const double hz = 0.5 * (z1 - z0), zc = z0 + hz;
  for (size_t k = 0; k < v.size(); ++k) {
    const double j = 0.78 + 0.4 * ((hash2(int(k), int(seed), 5u) & 1023) / 1023.0);
    Eigen::Vector3d p = v[k].normalized() * j;
    p.z() = std::max(p.z(), -0.55);
    v[k] = Eigen::Vector3d(x + r * p.x(), y + r * p.y(), zc + hz * p.z());
  }
  const Eigen::Vector3d ctr(x, y, zc);
  for (const auto& tri3 : f) {
    Eigen::Vector3d a = v[size_t(tri3[0])], b = v[size_t(tri3[1])], c = v[size_t(tri3[2])];
    if ((b - a).cross(c - a).dot((a + b + c) / 3.0 - ctr) < 0) std::swap(b, c);
    tri(a, b, c, col);
  }
}

std::vector<Marker> terrainMarkers(const Terrain& t, double fine, double coarse, double tile, const std::string& frame) {
  std::vector<Marker> out;
  const double x1 = t.ox + (t.nx - 1) * t.res, y1 = t.oy + (t.ny - 1) * t.res;
  int id = 0;
  auto vcol = [&](double x, double y, double e) {
    // Material colour blended over a small neighbourhood: soft road edges instead of a staircase.
    std::array<float, 3> mc{0, 0, 0};
    for (int dj = -1; dj <= 1; ++dj)
      for (int di = -1; di <= 1; ++di) {
        const auto& c = materialInfo(t.material(x + 0.6 * di * e, y + 0.6 * dj * e)).color;
        for (int k = 0; k < 3; ++k) mc[size_t(k)] += c[size_t(k)] / 9.f;
      }
    const double dzdx = (t.height(x + e, y) - t.height(x - e, y)) / (2 * e);
    const double dzdy = (t.height(x, y + e) - t.height(x, y - e)) / (2 * e);
    const Eigen::Vector3d n = Eigen::Vector3d(-dzdx, -dzdy, 1.0).normalized();
    const double var = (0.86 + 0.14 * fbm(x / 18, y / 18, 77)) * (0.95 + 0.05 * fbm(x / 3.5, y / 3.5, 91));
    const double k = var * (1.0 - std::clamp((1.0 - n.z()) * 3.0, 0.0, 0.3));
    return shade(Color{float(mc[0] * k), float(mc[1] * k), float(mc[2] * k), 1.f}, n, 0.55);
  };
  const int step = std::max(1, int(std::lround(t.res > 0 ? 1.0 : 1.0)));
  for (double ty = t.oy; ty < y1 - 1e-6; ty += tile)
    for (double tx = t.ox; tx < x1 - 1e-6; tx += tile) {
      const double ex = std::min(tx + tile, x1), ey = std::min(ty + tile, y1);
      double lap = 0;  // relief detector: max |discrete Laplacian| at the native resolution
      for (double y = ty + t.res; y < ey - t.res; y += t.res * step)
        for (double x = tx + t.res; x < ex - t.res; x += t.res * step)
          lap = std::max(lap, std::abs(4 * t.height(x, y) - t.height(x + t.res, y) - t.height(x - t.res, y) -
                                       t.height(x, y + t.res) - t.height(x, y - t.res)));
      const double dres = lap > 0.10 ? fine : coarse;
      MeshBuilder mb(frame, "terrain", id++);
      std::vector<double> xs, ys;
      for (double x = tx; x < ex - 1e-6; x += dres) xs.push_back(x);
      for (double y = ty; y < ey - 1e-6; y += dres) ys.push_back(y);
      xs.push_back(ex), ys.push_back(ey);
      for (size_t j = 0; j + 1 < ys.size(); ++j)
        for (size_t i = 0; i + 1 < xs.size(); ++i) {
          const double xa = xs[i], xb = xs[i + 1], ya = ys[j], yb = ys[j + 1];
          const Eigen::Vector3d p00(xa, ya, t.height(xa, ya)), p10(xb, ya, t.height(xb, ya));
          const Eigen::Vector3d p11(xb, yb, t.height(xb, yb)), p01(xa, yb, t.height(xa, yb));
          const Color c00 = vcol(xa, ya, dres), c10 = vcol(xb, ya, dres), c11 = vcol(xb, yb, dres), c01 = vcol(xa, yb, dres);
          mb.tri(p00, p10, p11, c00, c10, c11);
          mb.tri(p00, p11, p01, c00, c11, c01);
        }
      // Skirts hide T-junction cracks where a fine tile meets a coarse one.
      auto skirt = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        const Color c = vcol(a.x(), a.y(), dres);
        const Eigen::Vector3d da(a.x(), a.y(), a.z() - 0.8), db(b.x(), b.y(), b.z() - 0.8);
        mb.tri(a, b, db, c, c, c), mb.tri(a, db, da, c, c, c);
      };
      for (size_t i = 0; i + 1 < xs.size(); ++i) {
        skirt({xs[i], ty, t.height(xs[i], ty)}, {xs[i + 1], ty, t.height(xs[i + 1], ty)});
        skirt({xs[i + 1], ey, t.height(xs[i + 1], ey)}, {xs[i], ey, t.height(xs[i], ey)});
      }
      for (size_t j = 0; j + 1 < ys.size(); ++j) {
        skirt({ex, ys[j], t.height(ex, ys[j])}, {ex, ys[j + 1], t.height(ex, ys[j + 1])});
        skirt({tx, ys[j + 1], t.height(tx, ys[j + 1])}, {tx, ys[j], t.height(tx, ys[j])});
      }
      out.push_back(mb.marker());
    }
  return out;
}

std::vector<Marker> objectMarkers(const World& w, const std::string& frame) {
  std::vector<Marker> out;
  int id = 0;
  MeshBuilder mb(frame, "objects", id++);
  auto flush = [&](bool force) {
    if (mb.triangles() > 25000 || (force && mb.triangles() > 0)) out.push_back(mb.marker()), mb = MeshBuilder(frame, "objects", id++);
  };
  for (const auto& b : w.boxes)
    if (b.visible) mb.box(b), flush(false);
  for (const auto& b : w.visuals) mb.box(b), flush(false);
  uint32_t seed = 1;
  for (const auto& c : w.cylinders) {
    if (c.rock) mb.rock(c.x, c.y, c.z0, c.z1, c.r, c.color, seed++);
    else mb.cylinder(c.x, c.y, c.z0, c.z1, c.r, c.color);
    flush(false);
  }
  flush(true);
  return out;
}

std::vector<Marker> actorMarkers(const World& w) {
  std::vector<Marker> out;
  for (size_t i = 0; i < w.actors.size(); ++i) {
    MeshBuilder mb("actor_" + std::to_string(i), "actors", int(i));
    for (const auto& b : w.actors[i].parts) mb.box(b);
    mb.marker().frame_locked = true;
    out.push_back(mb.marker());
  }
  return out;
}

std::vector<Box> egoBodyBoxes(const VehicleParams& v) {
  auto bx = [](double x0, double x1, double y0, double y1, double z0, double z1, Color c) {
    Box b;
    b.c = Eigen::Vector3d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
    b.half = Eigen::Vector3d((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2);
    b.color = c, b.refl = 0.5f, b.label = uint8_t(Label::kEgo), b.in_map = false;
    return b;
  };
  const double hw = v.width / 2;
  if (v.name == "container_agv") {
    const Color plat{0.20f, 0.30f, 0.42f, 1.f}, cont{0.86f, 0.46f, 0.12f, 1.f}, mast{0.25f, 0.25f, 0.27f, 1.f};
    return {bx(-v.rear, v.front, -hw, hw, 0.55, 1.55, plat), bx(-2.6, 9.6, -1.22, 1.22, 1.6, 4.19, cont),
            bx(9.7, 10.0, -0.2, 0.2, 1.55, 4.85, mast)};
  }
  const Color yel{0.97f, 0.70f, 0.08f, 1.f}, dark{0.16f, 0.16f, 0.17f, 1.f}, glass{0.20f, 0.27f, 0.34f, 1.f};
  return {bx(-v.rear, v.wheelbase - 0.4, -hw, hw, 1.6, 1.95, yel),                      // bed floor
          bx(-v.rear, v.wheelbase - 0.4, hw - 0.3, hw, 1.95, 3.9, yel),                // bed side L
          bx(-v.rear, v.wheelbase - 0.4, -hw, -hw + 0.3, 1.95, 3.9, yel),              // bed side R
          bx(v.wheelbase - 0.7, v.wheelbase - 0.4, -hw, hw, 1.95, 3.9, yel),           // headboard
          bx(v.wheelbase - 0.4, v.front - 0.6, -hw, hw, 3.6, 3.9, yel),                // canopy
          bx(-1.0, v.wheelbase + 1.2, -1.0, 1.0, 0.9, 1.6, dark),                      // frame
          bx(v.wheelbase - 0.2, v.front, -1.3, 1.3, 1.0, 2.6, yel),                    // engine/hood
          bx(v.wheelbase + 0.2, v.wheelbase + 1.8, 0.3, hw - 0.1, 2.0, 3.45, glass)};  // cab
}

std::vector<Marker> egoMarkers(const VehicleParams& v, const std::vector<Eigen::Isometry3d>& lidar_mounts,
                               const std::string& frame, const std::string& ns, float alpha) {
  MeshBuilder mb(frame, ns, 0);
  for (Box b : egoBodyBoxes(v)) b.color[3] = alpha, mb.box(b);
  const Color tire{0.08f, 0.08f, 0.09f, alpha}, hub{0.45f, 0.45f, 0.47f, alpha};
  const bool agv = v.name == "container_agv";
  const double r = agv ? 0.55 : 1.05, wd = agv ? 0.5 : 0.85, yt = v.track / 2;
  std::vector<double> axles = agv ? std::vector<double>{-1.6, 0.0, v.wheelbase - 1.2, v.wheelbase}
                                  : std::vector<double>{0.0, v.wheelbase};
  for (double ax : axles)
    for (double s : {-1.0, 1.0}) {
      mb.wheel(Eigen::Vector3d(ax, s * yt, r), r, wd, tire, hub);
      if (!agv && ax == 0.0) mb.wheel(Eigen::Vector3d(ax, s * (yt - wd - 0.05), r), r, wd, tire, hub);  // duals
    }
  for (const auto& T : lidar_mounts) {
    const Eigen::Vector3d p = T.translation();
    mb.cylinder(p.x(), p.y(), p.z() - 0.12, p.z() + 0.1, 0.11, Color{0.1f, 0.1f, 0.1f, alpha}, 12);
    mb.cylinder(p.x(), p.y(), p.z() - 0.03, p.z() + 0.03, 0.115, Color{0.2f, 0.75f, 0.95f, alpha}, 12);
  }
  Marker m = mb.marker();
  m.frame_locked = true;
  return {m};
}

Marker dustMarker(const World& w, const std::string& frame, FastRng& rng) {
  Marker m;
  m.header.frame_id = frame;
  m.ns = "dust", m.id = 0, m.type = Marker::POINTS, m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = 1.1;
  for (const auto& p : w.dust) {
    const double dens = p.currentDensity();
    const int n = int(std::clamp(10.0 * p.r, 15.0, 90.0));
    for (int k = 0; k < n; ++k) {
      Eigen::Vector3d u;
      do u = Eigen::Vector3d(2 * rng.uniform() - 1, 2 * rng.uniform() - 1, 2 * rng.uniform() - 1);
      while (u.squaredNorm() > 1.0);
      m.points.push_back(toPt(p.c + p.r * u));
      std_msgs::msg::ColorRGBA c;
      c.r = 0.78f, c.g = 0.68f, c.b = 0.52f;
      c.a = float(std::clamp(dens * 1.4 * (1.0 - 0.7 * u.squaredNorm()), 0.03, 0.4));
      m.colors.push_back(c);
    }
  }
  return m;
}

}  // namespace iad::sim
