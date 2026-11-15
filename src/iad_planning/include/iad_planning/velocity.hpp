// Time parameterisation of a geometric path: speed limits (gear, curvature, terrain), a stop at every
// cusp and at the goal, then forward/backward passes with acceleration and deceleration limits. ROS-free.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "iad_planning/common.hpp"
#include "iad_planning/hybrid_astar.hpp"

namespace iad::planning {

struct TrajPoint {
  double x = 0, y = 0, yaw = 0, v = 0, a = 0, kappa = 0, s = 0;
  int dir = 1;
};
using Trajectory = std::vector<TrajPoint>;

struct SpeedParams {
  double v_fwd = 6.0, v_rev = 1.6, v_min = 0.5;
  // Steering-rate limit: where curvature changes quickly (a Reeds-Shepp arc starts), slow down so the
  // rate-limited hydraulic steering can follow: dkappa/dt = dkappa/ds * v <= steer_rate / wheelbase.
  double steer_rate = 0.35, wheelbase = 4.3;
  double a_lat = 0.9;            // [m/s^2] curve limit (high centre of gravity, loose ground)
  double a_acc = 0.5, a_dec = 0.6;
  double terrain_slow = 0.5;     // speed factor at terrain cost 99
};

inline Trajectory timeParameterize(const Path& p, const CostGrid& map, const SpeedParams& sp, double v0 = 0.0) {
  Trajectory t(p.size());
  if (p.empty()) return t;
  for (size_t i = 0; i < p.size(); ++i) {
    t[i].x = p[i].x, t[i].y = p[i].y, t[i].yaw = p[i].yaw, t[i].dir = i + 1 < p.size() ? p[i + 1].dir : p[i].dir;
    if (i) t[i].s = t[i - 1].s + std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
  }
  for (size_t i = 1; i + 1 < p.size(); ++i) {
    const double ds = t[i + 1].s - t[i - 1].s;
    t[i].kappa = ds > 1e-6 ? wrapAngle(t[i + 1].yaw - t[i - 1].yaw) / ds : 0.0;
  }
  std::vector<double> ks(p.size(), 0.0), dk(p.size(), 0.0);  // curvature smoothed over +-2 m, and its rate
  for (size_t i = 0; i < p.size(); ++i) {
    double sum = 0;
    int n = 0;
    for (size_t j = i > 4 ? i - 4 : 0; j <= std::min(p.size() - 1, i + 4); ++j) sum += t[j].kappa, ++n;
    ks[i] = sum / std::max(1, n);
  }
  for (size_t i = 1; i + 1 < p.size(); ++i) dk[i] = std::abs(ks[i + 1] - ks[i - 1]) / std::max(1e-3, t[i + 1].s - t[i - 1].s);
  std::vector<double> vmax(p.size());
  for (size_t i = 0; i < p.size(); ++i) {
    double v = t[i].dir > 0 ? sp.v_fwd : sp.v_rev;
    v = std::min(v, std::sqrt(sp.a_lat / std::max(1e-4, std::abs(t[i].kappa))));
    v = std::min(v, sp.steer_rate / (sp.wheelbase * std::max(1e-4, dk[i])));
    const double c = std::max<int>(0, map.empty() ? 0 : map.valueAt(t[i].x, t[i].y, 0));
    v *= 1.0 - (1.0 - sp.terrain_slow) * c / 99.0;
    vmax[i] = std::max(v, sp.v_min);
  }
  // Stop at the goal and at every gear change (the cusp point belongs to both segments).
  vmax.back() = 0;
  for (size_t i = 1; i + 1 < p.size(); ++i)
    if (p[i + 1].dir != p[i].dir) vmax[i] = 0;
  vmax[0] = std::min(vmax[0], std::max(std::abs(v0), sp.v_min));
  for (size_t i = p.size() - 1; i-- > 0;)  // backward: braking
    vmax[i] = std::min(vmax[i], std::sqrt(vmax[i + 1] * vmax[i + 1] + 2 * sp.a_dec * (t[i + 1].s - t[i].s)));
  for (size_t i = 1; i < p.size(); ++i)  // forward: acceleration
    vmax[i] = std::min(vmax[i], std::sqrt(vmax[i - 1] * vmax[i - 1] + 2 * sp.a_acc * (t[i].s - t[i - 1].s)));
  for (size_t i = 0; i < p.size(); ++i) {
    t[i].v = t[i].dir * vmax[i];
    if (i + 1 < p.size()) {
      const double ds = std::max(1e-3, t[i + 1].s - t[i].s);
      t[i].a = (vmax[i + 1] * vmax[i + 1] - vmax[i] * vmax[i]) / (2 * ds);
    }
  }
  return t;
}

}  // namespace iad::planning
