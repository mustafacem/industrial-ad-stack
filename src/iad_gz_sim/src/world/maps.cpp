#include "iad_gz_sim/maps.hpp"

#include <unordered_map>

#include "iad_gz_sim/common/voxel.hpp"

namespace iad::sim {

Cloud buildPointcloudMap(const World& w, double ts, double os, double voxel) {
  struct Acc {
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    double i = 0;
    int n = 0;
  };
  std::unordered_map<int64_t, Acc> vox;
  const double inv = 1.0 / voxel;
  auto add = [&](const Eigen::Vector3d& p, double refl) {
    auto& a = vox[voxelOf(p, inv)];
    a.s += p, a.i += refl * 255.0, ++a.n;
  };
  const Terrain& t = w.terrain;
  const double x1 = t.ox + (t.nx - 1) * t.res, y1 = t.oy + (t.ny - 1) * t.res;
  for (double y = t.oy; y <= y1; y += ts)
    for (double x = t.ox; x <= x1; x += ts) {
      const uint8_t m = t.material(x, y);
      if (m != kWater) add({x, y, t.height(x, y)}, materialInfo(m).reflectivity);
    }
  for (const auto& b : w.boxes) {
    if (!b.in_map) continue;
    const double c = std::cos(b.yaw), s = std::sin(b.yaw);
    auto P = [&](double lx, double ly, double lz) {
      return Eigen::Vector3d(b.c.x() + c * lx - s * ly, b.c.y() + s * lx + c * ly, b.c.z() + lz);
    };
    const Eigen::Vector3d h = b.half;
    for (double z = -h.z(); z <= h.z() + 1e-9; z += os) {
      for (double x = -h.x(); x <= h.x() + 1e-9; x += os) add(P(x, -h.y(), z), b.refl), add(P(x, h.y(), z), b.refl);
      for (double y = -h.y(); y <= h.y() + 1e-9; y += os) add(P(-h.x(), y, z), b.refl), add(P(h.x(), y, z), b.refl);
    }
    for (double x = -h.x(); x <= h.x() + 1e-9; x += os)
      for (double y = -h.y(); y <= h.y() + 1e-9; y += os) add(P(x, y, h.z()), b.refl);
  }
  for (const auto& cy : w.cylinders) {
    if (!cy.in_map) continue;
    const int na = std::max(8, int(2 * kPi * cy.r / os));
    for (double z = cy.z0; z <= cy.z1; z += os)
      for (int k = 0; k < na; ++k) {
        const double a = 2 * kPi * k / na;
        add({cy.x + cy.r * std::cos(a), cy.y + cy.r * std::sin(a), z}, cy.refl);
      }
  }
  Cloud out;
  out.reserve(vox.size());
  for (const auto& kv : vox) {
    const Eigen::Vector3d p = kv.second.s / kv.second.n;
    out.push_back({float(p.x()), float(p.y()), float(p.z()), float(kv.second.i / kv.second.n), 0, 0, 0});
  }
  return out;
}

CostGrid buildCostmap(const World& w, double res, double clearance) {
  const Terrain& t = w.terrain;
  const double W = (t.nx - 1) * t.res, H = (t.ny - 1) * t.res;
  CostGrid g(int(W / res), int(H / res), res, t.ox, t.oy, 0);
  for (int j = 0; j < g.h; ++j)
    for (int i = 0; i < g.w; ++i) {
      const double x = g.cx(i), y = g.cy(j);
      const uint8_t m = t.material(x, y);
      if (m == kWater) {
        g.at(i, j) = kLethal;
        continue;
      }
      const Eigen::Vector3d n = t.normal(x, y);
      const double grade = std::sqrt(std::max(0.0, 1.0 - n.z() * n.z())) / n.z();
      const double h0 = t.height(x, y);
      const double rough = std::abs(h0 - 0.25 * (t.height(x + 1, y) + t.height(x - 1, y) + t.height(x, y + 1) + t.height(x, y - 1)));
      if (grade > 0.32) {
        g.at(i, j) = kLethal;
        continue;
      }
      const double cost = materialInfo(m).cost + 70.0 * sq(grade / 0.32) + std::min(35.0, 180.0 * rough);
      g.at(i, j) = int8_t(std::clamp(cost, 0.0, 99.0));
    }
  auto ground = [&](double x, double y) { return t.height(x, y); };
  for (const auto& b : w.boxes) {
    if (!b.in_map) continue;
    if (b.c.z() - b.half.z() - ground(b.c.x(), b.c.y()) > clearance) continue;  // overhead structure
    const double c = std::cos(b.yaw), s = std::sin(b.yaw), r = b.radius() + res;
    int i0, j0, i1, j1;
    g.toCell(b.c.x() - r, b.c.y() - r, i0, j0), g.toCell(b.c.x() + r, b.c.y() + r, i1, j1);
    for (int j = std::max(0, j0); j <= std::min(g.h - 1, j1); ++j)
      for (int i = std::max(0, i0); i <= std::min(g.w - 1, i1); ++i) {
        const double dx = g.cx(i) - b.c.x(), dy = g.cy(j) - b.c.y();
        if (std::abs(c * dx + s * dy) <= b.half.x() + 0.5 * res && std::abs(-s * dx + c * dy) <= b.half.y() + 0.5 * res)
          g.at(i, j) = kLethal;
      }
  }
  for (const auto& cy : w.cylinders) {
    if (!cy.in_map || cy.z1 - ground(cy.x, cy.y) < 0.25) continue;
    const double r = cy.r + 0.5 * res;
    int i0, j0, i1, j1;
    g.toCell(cy.x - r, cy.y - r, i0, j0), g.toCell(cy.x + r, cy.y + r, i1, j1);
    for (int j = std::max(0, j0); j <= std::min(g.h - 1, j1); ++j)
      for (int i = std::max(0, i0); i <= std::min(g.w - 1, i1); ++i)
        if (std::hypot(g.cx(i) - cy.x, g.cy(j) - cy.y) <= r) g.at(i, j) = kLethal;
  }
  return g;
}

}  // namespace iad::sim
