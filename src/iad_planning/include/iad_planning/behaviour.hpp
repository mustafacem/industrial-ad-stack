// Behaviour layer: speed caps on the current trajectory from what perception reports (tracked objects with
// velocity, dust echoes), so the truck drives like a site rule book says:
//   dust    - slow down while the lane ahead is full of dust (visibility), keep going
//   follow  - a slower vehicle ahead in the lane: keep a time gap, never closer than follow_min
//   passing - an oncoming vehicle in the other lane: pass it at reduced speed
//   yield   - a vehicle about to cross the path: stop before its crossing line until it is through
// Static obstacles stay with the occupancy stop / replan logic. ROS-free.
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "iad_planning/velocity.hpp"

namespace iad::planning {

struct Agent {  // a moving object in the map frame (box yaw: a truck seen from behind is a 4 x 0.2 m box across it)
  double x = 0, y = 0, vx = 0, vy = 0, length = 4, width = 2, yaw = 0;
};

struct BehaviourParams {
  double dust_range = 35.0;                 // [m] ahead of the front, inside the lane corridor
  int dust_light = 150, dust_heavy = 600;   // dust echoes per scan in that corridor
  double dust_v_light = 3.0, dust_v_heavy = 2.0;
  double follow_gap = 10.0, follow_time = 2.0, follow_k = 0.4, follow_min = 8.0;
  double pass_lateral = 13.0, pass_range = 45.0, pass_v = 4.0;
  double yield_horizon = 6.0, yield_range = 45.0, yield_margin = 5.0;
  double moving = 1.0;                      // [m/s] slower objects are static (occupancy logic)
  double a_comf = 0.6, a_stop = 1.0;
};

struct Behaviour {
  std::string name = "cruise";  // cruise | dust | follow | passing | yield
  double v_cap = 1e9;           // [m/s]
  double s_stop = 1e9;          // [m] trajectory arc length where the rear axle must stop
  double info = 0;              // gap (follow), echoes (dust), distance ahead (passing, yield)
  double v_eff = 1e9;           // the resulting speed limit at the current position
};

// t: remaining trajectory (t.front() = current position); half_w / front / rear: ego footprint.
inline Behaviour decideBehaviour(const Trajectory& t, double half_w, double front, double rear, double v_ego,
                                 const std::vector<Agent>& agents, int dust, const BehaviourParams& bp) {
  Behaviour best;
  if (t.size() < 2 || t.front().dir < 0) return best;
  const double s0 = t.front().s;
  double v_ref = 0;  // what the plan asks for over the next 30 m: a cap above it does not change anything
  for (size_t i = 0; i < t.size() && t[i].s - s0 < 30.0; ++i) v_ref = std::max(v_ref, std::abs(t[i].v));
  auto consider = [&](const char* name, double v_cap, double s_stop, double info) {
    const double v_eff = std::min(v_cap, s_stop > 1e8 ? 1e9 : std::sqrt(2 * bp.a_stop * std::max(0.0, s_stop - s0)));
    if (v_eff < best.v_eff && v_eff < v_ref - 0.25) best = {name, v_cap, s_stop, info, v_eff};
  };
  if (dust >= bp.dust_heavy) consider("dust", bp.dust_v_heavy, 1e9, dust);
  else if (dust >= bp.dust_light) consider("dust", bp.dust_v_light, 1e9, dust);

  for (const Agent& a : agents) {
    // Nearest point of the forward stretch (60 m) and the agent in that point's frame.
    double bd = 1e18, s_rel = 0, d = 0, va = 0, vl = 0, pyaw = 0;
    for (size_t i = 0; i + 1 < t.size() && t[i].dir > 0 && t[i].s - s0 < 60.0; ++i) {
      const double c = std::cos(t[i].yaw), sn = std::sin(t[i].yaw), dx = a.x - t[i].x, dy = a.y - t[i].y;
      const double u = c * dx + sn * dy, l = -sn * dx + c * dy;
      const double seg = t[i + 1].s - t[i].s, uc = std::clamp(u, i ? 0.0 : -1e9, seg);
      const double dist = std::hypot(u - uc, l);
      if (dist < bd) bd = dist, s_rel = t[i].s - s0 + u, d = l, va = c * a.vx + sn * a.vy, vl = -sn * a.vx + c * a.vy, pyaw = t[i].yaw;
    }
    if (bd > 25.0) continue;
    const double ca = std::abs(std::cos(a.yaw - pyaw)), sa = std::abs(std::sin(a.yaw - pyaw));
    const double lo = ca * a.length / 2 + sa * a.width / 2, ho = sa * a.length / 2 + ca * a.width / 2;  // along / across
    const double corridor = half_w + ho + 0.5;
    const bool in_lane = std::abs(d) < corridor;
    if (in_lane && va > bp.moving && s_rel > 0) {  // lead vehicle
      const double gap = s_rel - lo - front, want = bp.follow_gap + bp.follow_time * va;
      consider("follow", std::max(0.0, va + bp.follow_k * (gap - want)), s0 + s_rel - lo - bp.follow_min - front, gap);
    } else if (!in_lane && std::abs(d) < bp.pass_lateral && va < -bp.moving && s_rel > -lo - rear && s_rel < bp.pass_range) {
      consider("passing", bp.pass_v, 1e9, s_rel);
    }
    // Crossing: moving mostly across the path, towards it (or already in it), in the way when the ego gets there.
    if (std::abs(vl) > std::max(bp.moving, std::abs(va)) && d * vl < 0 && s_rel > -lo && s_rel < bp.yield_range) {
      const double cross = half_w + ho + 1.0, t_in = std::max(0.0, std::abs(d) - cross) / std::abs(vl);
      const double t_out = (std::abs(d) + cross) / std::abs(vl);
      const double t_ego = std::max(0.0, s_rel - ho - front) / std::max(v_ego, 1.0);
      if (t_in < bp.yield_horizon && t_ego < t_out + 1.0)
        consider("yield", 1e9, s0 + s_rel - ho - bp.yield_margin - front, s_rel);
    }
  }
  return best;
}

// Caps the forward stretch: decelerate at a_comf from v_now to v_cap, stop at s_stop with a_stop.
inline Trajectory applyBehaviour(Trajectory t, const Behaviour& b, double v_now, const BehaviourParams& bp) {
  if (t.empty() || b.v_eff > 1e8) return t;
  const double s0 = t.front().s;
  size_t end = 0;
  for (; end < t.size() && t[end].dir > 0; ++end) {
    auto& p = t[end];
    double v = std::min(p.v, std::sqrt(std::max(b.v_cap * b.v_cap, v_now * v_now - 2 * bp.a_comf * (p.s - s0))));
    if (b.s_stop < 1e8) v = p.s >= b.s_stop ? 0.0 : std::min(v, std::sqrt(2 * bp.a_stop * (b.s_stop - p.s)));
    p.v = v;
  }
  for (size_t i = 0; i + 1 < t.size(); ++i) {
    const double ds = std::max(1e-3, t[i + 1].s - t[i].s);
    t[i].a = (t[i + 1].v * t[i + 1].v - t[i].v * t[i].v) / (2 * ds);
  }
  return t;
}

}  // namespace iad::planning
