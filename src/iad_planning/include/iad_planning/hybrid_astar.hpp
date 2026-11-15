// Hybrid A* free-space planner (no lanes, just a site costmap), same family as Autoware's
// freespace_planner. ROS-free. Builds on offroad_av v1, adding:
//  - Reeds-Shepp analytic expansion: every few expansions, try an exact RS "shot" to the goal pose;
//    docking precision comes for free and the last metres need no lattice search.
//  - heuristic = max(obstacle-aware 2D Dijkstra (weighted), Reeds-Shepp distance ignoring obstacles)
//  - soft clearance cost: paths keep away from berms, stacks and crane legs when there is room.
//  - region of interest: the search runs on a crop of the site map around start and goal.
#pragma once
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "iad_planning/common.hpp"
#include "iad_planning/reeds_shepp.hpp"

namespace iad::planning {

struct PathPoint {
  double x = 0, y = 0, yaw = 0;
  int dir = 1;  // +1 forward, -1 reverse (motion into this point)
};
using Path = std::vector<PathPoint>;

struct PlannerParams {
  double xy_res = 1.0;
  int yaw_bins = 72;
  double step = 1.6, sub = 0.5;
  int n_steer = 7;
  double reverse_mult = 1.8, gear_change = 18.0, steer_cost = 1.0, steer_change = 2.5;
  double terrain_weight = 2.0;   // cost multiplier = 1 + w * cell_cost / 100
  double clearance_weight = 1.5, clearance_safe = 2.5;  // soft cost when side clearance < safe [m]
  double h_weight = 1.4;
  double pos_tol = 0.4, yaw_tol = 0.04;
  double margin = 0.4;
  double steer_headroom = 0.85;
  int max_expansions = 200000;
  int rs_every = 6;
  double rs_max_dist = 70.0;
  double roi_margin = 50.0;
};

struct PlanStats {
  int expansions = 0;
  bool rs_shot = false;
  double ms = 0;
  std::string status = "idle";
  std::vector<std::pair<PathPoint, PathPoint>> tree;  // sampled search edges, for visualisation
  size_t rs_begin = 0;                                 // index where the RS shot starts in the path
};

class HybridAStar {
 public:
  HybridAStar(const VehicleParams& v, const PlannerParams& p) : v_(v), p_(p), rs_(1.0) {
    v_.max_steer *= p_.steer_headroom;
    rs_ = ReedsShepp(v_.wheelbase / std::tan(v_.max_steer));
  }
  const VehicleParams& vehicle() const { return v_; }
  double turnRadius() const { return v_.wheelbase / std::tan(v_.max_steer); }

  Path plan(const CostGrid& full, const Pose2& start, int start_dir, const Pose2& goal, PlanStats* st = nullptr) const {
    Stopwatch sw;
    PlanStats local;
    PlanStats& S = st ? *st : local;
    S = PlanStats{};
    const CostGrid map = crop(full, start, goal);
    int gi, gj;
    if (!map.toCell(goal.x, goal.y, gi, gj) || footprintCollides(map, goal.x, goal.y, goal.yaw, v_, p_.margin)) {
      S.status = "goal_in_collision", S.ms = sw.ms();
      return {};
    }
    if (footprintCollides(map, start.x, start.y, start.yaw, v_, 0.0)) {
      S.status = "start_in_collision", S.ms = sw.ms();
      return {};
    }
    std::vector<int> lethal;
    for (int c = 0; c < int(map.data.size()); ++c) {
      const int i = c % map.w, j = c / map.w;
      if (map.data[size_t(c)] >= kLethal || i == 0 || j == 0 || i == map.w - 1 || j == map.h - 1) lethal.push_back(c);
    }
    const Grid<float> clear = dijkstra(map, lethal, nullptr);
    const Grid<float> h2d = dijkstra(map, {gj * map.w + gi}, &clear);
    const double ac = (v_.front - v_.rear) / 2;
    const double rc = std::hypot((v_.front + v_.rear) / 2 + p_.margin, v_.width / 2 + p_.margin) + 2.0 * map.res;
    auto isFree = [&](double x, double y, double yaw) {
      if (0.92 * clear.valueAt(x + ac * std::cos(yaw), y + ac * std::sin(yaw), 0.f) > rc) return true;
      return !footprintCollides(map, x, y, yaw, v_, p_.margin);
    };
    const int W = int(std::ceil(map.w * map.res / p_.xy_res)), H = int(std::ceil(map.h * map.res / p_.xy_res));
    auto key = [&](double x, double y, double yaw) -> long {
      const int i = int(std::floor((x - map.ox) / p_.xy_res)), j = int(std::floor((y - map.oy) / p_.xy_res));
      const int k = int(std::floor((wrapAngle(yaw) + kPi) / (2 * kPi) * p_.yaw_bins)) % p_.yaw_bins;
      return (i < 0 || j < 0 || i >= W || j >= H) ? -1 : (long(j) * W + i) * p_.yaw_bins + k;
    };
    auto heur = [&](double x, double y, double yaw) {
      const double h = p_.h_weight * h2d.valueAt(x, y, std::numeric_limits<float>::infinity());
      if (!std::isfinite(h)) return h;
      return std::max(h, rs_.distance(x, y, yaw, goal.x, goal.y, goal.yaw));
    };
    struct Node { Pose2 p; double g; int parent; double steer; int dir; };
    std::vector<Node> nodes{{start, 0.0, -1, 0.0, start_dir}};
    std::vector<double> best(size_t(W) * size_t(H) * size_t(p_.yaw_bins), kInf);
    const long k0 = key(start.x, start.y, start.yaw);
    if (k0 < 0) {
      S.status = "start_off_map", S.ms = sw.ms();
      return {};
    }
    best[size_t(k0)] = 0;
    using QE = std::pair<double, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    open.push({heur(start.x, start.y, start.yaw), 0});
    while (!open.empty() && S.expansions < p_.max_expansions) {
      const int idx = open.top().second;
      open.pop();
      const Node n = nodes[size_t(idx)];
      if (n.g > best[size_t(key(n.p.x, n.p.y, n.p.yaw))]) continue;
      ++S.expansions;
      const double dgoal = std::hypot(n.p.x - goal.x, n.p.y - goal.y);
      if (dgoal < p_.pos_tol && std::abs(wrapAngle(n.p.yaw - goal.yaw)) < p_.yaw_tol) {
        S.status = "ok", S.ms = sw.ms(), S.rs_begin = SIZE_MAX;
        const Path raw = reconstruct(nodes, idx);
        return smooth(shortcut(raw, raw.size(), isFree), isFree, SIZE_MAX);
      }
      if (S.expansions % p_.rs_every == 0 && dgoal < p_.rs_max_dist) {  // analytic expansion
        // Gear changes are for docking (goal heading turned around), not for a shunt at a forward arrival:
        // otherwise the shot is restricted to the current gear (a single-gear Reeds-Shepp word).
        const bool turnaround = std::abs(wrapAngle(goal.yaw - n.p.yaw)) > kPi / 2;
        const bool may_shunt = turnaround && dgoal < 25.0;
        double len = 0;
        const auto shot = rs_.sample(n.p.x, n.p.y, n.p.yaw, goal.x, goal.y, goal.yaw, p_.sub, &len,
                                     may_shunt ? 0 : (n.dir != 0 ? n.dir : 1));
        bool ok = !shot.empty() && (may_shunt || len < 3.0 * dgoal + 10.0);
        for (const auto& q : shot) ok = ok && isFree(q.x, q.y, q.yaw);
        if (ok) {
          Path path = reconstruct(nodes, idx);
          const size_t rs_begin = path.size();
          for (const auto& q : shot) path.push_back({q.x, q.y, q.yaw, q.dir});
          Path sc = shortcut(path, rs_begin, isFree);
          // the RS tail is kept verbatim unless a same-gear goal connection replaced it
          const size_t tail = path.size() - rs_begin;
          const size_t rs_new = sc.size() >= tail && sc.back().x == path.back().x && sc.size() - tail > 0 &&
                                        sc[sc.size() - tail].x == path[rs_begin].x ? sc.size() - tail : sc.size();
          S.status = "ok", S.rs_shot = true, S.ms = sw.ms(), S.rs_begin = rs_new;
          return smooth(sc, isFree, rs_new);
        }
      }
      for (int dir : {1, -1})
        for (int s = 0; s < p_.n_steer; ++s) {
          const double steer = -v_.max_steer + 2 * v_.max_steer * s / (p_.n_steer - 1);
          const auto arc = drive(n.p, steer, dir);
          bool free = true;
          for (const auto& q : arc) free = free && isFree(q.x, q.y, q.yaw);
          if (!free) continue;
          const Pose2 q{arc.back().x, arc.back().y, arc.back().yaw};
          const long k = key(q.x, q.y, q.yaw);
          if (k < 0) continue;
          const double hq = heur(q.x, q.y, q.yaw);
          if (!std::isfinite(hq)) continue;
          const double terrain = 1.0 + p_.terrain_weight * std::max<int>(0, map.valueAt(q.x, q.y, 0)) / 100.0;
          const double side = clear.valueAt(q.x + ac * std::cos(q.yaw), q.y + ac * std::sin(q.yaw), 0.f) - v_.width / 2;
          const double clr = p_.clearance_weight * std::max(0.0, p_.clearance_safe - side);
          const double g = n.g + p_.step * (terrain + clr) * (dir < 0 ? p_.reverse_mult : 1.0) +
                           p_.steer_cost * std::abs(steer) * p_.step + p_.steer_change * std::abs(steer - n.steer) +
                           (n.dir != 0 && n.dir != dir ? p_.gear_change : 0.0);
          if (g >= best[size_t(k)]) continue;
          best[size_t(k)] = g;
          nodes.push_back({q, g, idx, steer, dir});
          open.push({g + hq, int(nodes.size()) - 1});
          if (nodes.size() % 7 == 0)
            S.tree.push_back({{n.p.x, n.p.y, n.p.yaw, dir}, {q.x, q.y, q.yaw, dir}});
        }
    }
    S.status = open.empty() ? "no_path" : "timeout", S.ms = sw.ms();
    return {};
  }

 private:
  static constexpr double kInf = std::numeric_limits<double>::infinity();

  CostGrid crop(const CostGrid& g, const Pose2& a, const Pose2& b) const {
    const double m = p_.roi_margin;
    int i0, j0, i1, j1;
    g.toCell(std::min(a.x, b.x) - m, std::min(a.y, b.y) - m, i0, j0);
    g.toCell(std::max(a.x, b.x) + m, std::max(a.y, b.y) + m, i1, j1);
    i0 = std::clamp(i0, 0, g.w - 1), j0 = std::clamp(j0, 0, g.h - 1), i1 = std::clamp(i1, 0, g.w - 1), j1 = std::clamp(j1, 0, g.h - 1);
    CostGrid c(i1 - i0 + 1, j1 - j0 + 1, g.res, g.ox + i0 * g.res, g.oy + j0 * g.res, 0);
    for (int j = 0; j < c.h; ++j)
      for (int i = 0; i < c.w; ++i) c.at(i, j) = g.at(i0 + i, j0 + j);
    return c;
  }

  Path drive(const Pose2& from, double steer, int dir) const {
    const int n = int(std::ceil(p_.step / p_.sub));
    const double ds = dir * p_.step / n, kappa = std::tan(steer) / v_.wheelbase;
    Path out;
    double x = from.x, y = from.y, th = from.yaw;
    for (int i = 0; i < n; ++i) {
      if (std::abs(kappa) < 1e-9) {
        x += ds * std::cos(th), y += ds * std::sin(th);
      } else {
        const double dth = ds * kappa;
        x += (std::sin(th + dth) - std::sin(th)) / kappa;
        y += (std::cos(th) - std::cos(th + dth)) / kappa;
        th += dth;
      }
      out.push_back({x, y, wrapAngle(th), dir});
    }
    return out;
  }

  template <typename N>
  Path reconstruct(const std::vector<N>& nodes, int idx) const {
    std::vector<int> chain;
    for (int i = idx; i >= 0; i = nodes[size_t(i)].parent) chain.push_back(i);
    const auto& s = nodes[size_t(chain.back())].p;
    Path path{{s.x, s.y, s.yaw, chain.size() > 1 ? nodes[size_t(chain[chain.size() - 2])].dir : 1}};
    for (int c = int(chain.size()) - 2; c >= 0; --c) {
      const auto& n = nodes[size_t(chain[size_t(c)])];
      for (const auto& q : drive(nodes[size_t(n.parent)].p, n.steer, n.dir)) path.push_back(q);
    }
    return path;
  }

  // Lattice paths meander (weighted A* commits greedily, headings come in 5-degree bins). Shortcut:
  // from each pose, jump to the farthest later pose of the same gear that a single-gear Reeds-Shepp
  // curve reaches collision-free without being longer. Only the lattice part (before `end`) is touched.
  template <typename FreeFn>
  Path shortcut(const Path& p, size_t end, const FreeFn& isFree) const {
    end = std::min(end, p.size());
    if (end < 3) return p;
    std::vector<double> s(p.size(), 0.0);
    for (size_t i = 1; i < p.size(); ++i) s[i] = s[i - 1] + std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    Path out{p[0]};
    size_t i = 0;
    // Same-gear connection straight to the final pose (removes end-of-path shunts when one exists).
    auto toGoal = [&](size_t from) {
      if (s.back() - s[from] > 60.0 || from + 2 >= p.size()) return false;
      double len = 0;
      const auto c = rs_.sample(p[from].x, p[from].y, p[from].yaw, p.back().x, p.back().y, p.back().yaw, p_.sub, &len, p[from + 1].dir);
      if (c.empty() || len > s.back() - s[from] + 0.3) return false;
      for (const auto& q : c)
        if (q.dir != c.front().dir || q.dir != p[from + 1].dir || !isFree(q.x, q.y, q.yaw)) return false;
      for (const auto& q : c) out.push_back({q.x, q.y, q.yaw, q.dir});
      return true;
    };
    bool cusp_after = false;
    for (size_t k = 1; k < p.size(); ++k) cusp_after = cusp_after || p[k].dir != p[k - 1].dir;
    while (i + 1 < end) {
      if (cusp_after && toGoal(i)) return out;
      size_t seg_end = i + 1;  // same-gear stretch
      while (seg_end + 1 < end && p[seg_end + 1].dir == p[i + 1].dir) ++seg_end;
      bool jumped = false;
      for (size_t j = std::min(seg_end, i + 360); j > i + 6 && !jumped; j = j > 8 ? j - 8 : 0) {
        double len = 0;
        const auto c = rs_.sample(p[i].x, p[i].y, p[i].yaw, p[j].x, p[j].y, p[j].yaw, p_.sub, &len, p[j].dir);
        if (c.empty() || len > s[j] - s[i] + 0.3) continue;
        bool ok = true;
        for (const auto& q : c) ok = ok && q.dir == p[j].dir && isFree(q.x, q.y, q.yaw);
        if (!ok) continue;
        for (const auto& q : c) out.push_back({q.x, q.y, q.yaw, q.dir});
        i = j, jumped = true;
      }
      if (!jumped) out.push_back(p[++i]);
    }
    for (size_t k = end; k < p.size(); ++k) out.push_back(p[k]);
    return out;
  }

  // Gradient-descent smoothing that never moves a point into collision or above max curvature; cusps,
  // endpoints and the exact Reeds-Shepp tail stay fixed.
  template <typename FreeFn>
  Path smooth(const Path& in, const FreeFn& isFree, size_t rs_begin) const {
    const double w_data = 0.02, w_smooth = 0.3, kappa_max = 0.95 * std::tan(v_.max_steer) / v_.wheelbase;
    Path p = in;
    const size_t end = std::min(rs_begin, p.size());
    auto fixed = [&](size_t i) { return i == 0 || i + 1 >= end || p[i].dir != p[i + 1].dir; };
    auto heading = [&](size_t i) {
      const double m = std::atan2(p[i + 1].y - p[i - 1].y, p[i + 1].x - p[i - 1].x);
      return wrapAngle(p[i].dir > 0 ? m : m + kPi);
    };
    auto curvature = [&](size_t i) {
      if (i == 0 || i + 1 >= p.size()) return 0.0;
      const double a1 = std::atan2(p[i].y - p[i - 1].y, p[i].x - p[i - 1].x), a2 = std::atan2(p[i + 1].y - p[i].y, p[i + 1].x - p[i].x);
      const double ds = 0.5 * (std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y) + std::hypot(p[i + 1].x - p[i].x, p[i + 1].y - p[i].y));
      return std::abs(wrapAngle(a2 - a1)) / std::max(ds, 1e-6);
    };
    for (int it = 0; it < 300; ++it)
      for (size_t i = 1; i + 1 < end; ++i) {
        if (fixed(i) || p[i - 1].dir != p[i].dir) continue;
        const PathPoint old = p[i];
        auto kmax = [&] { return std::max({curvature(i - 1), curvature(i), curvature(i + 1)}); };
        const double k_before = kmax();
        p[i].x += w_data * (in[i].x - p[i].x) + w_smooth * (p[i - 1].x + p[i + 1].x - 2 * p[i].x);
        p[i].y += w_data * (in[i].y - p[i].y) + w_smooth * (p[i - 1].y + p[i + 1].y - 2 * p[i].y);
        p[i].yaw = heading(i);
        if (!isFree(p[i].x, p[i].y, p[i].yaw) || kmax() > std::max(kappa_max, k_before)) p[i] = old;
      }
    for (size_t i = 1; i + 1 < end; ++i)
      if (!fixed(i) && p[i - 1].dir == p[i].dir) p[i].yaw = heading(i);
    return p;
  }

  // 8-connected multi-source Dijkstra. clear == nullptr: distance to lethal cells (clearance map).
  // Otherwise: terrain-weighted cost-to-go from the goal through cells the truck's width fits.
  Grid<float> dijkstra(const CostGrid& map, const std::vector<int>& sources, const Grid<float>* clear) const {
    const bool heuristic = clear != nullptr;
    Grid<float> d(map.w, map.h, map.res, map.ox, map.oy, std::numeric_limits<float>::infinity());
    using QE = std::pair<float, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> q;
    for (int c : sources) d.data[size_t(c)] = 0, q.push({0.f, c});
    while (!q.empty()) {
      const auto [dc, c] = q.top();
      q.pop();
      const int i = c % map.w, j = c / map.w;
      if (dc > d.at(i, j)) continue;
      for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
          const int ni = i + di, nj = j + dj;
          if ((!di && !dj) || !map.inside(ni, nj) || (heuristic && clear->at(ni, nj) < v_.width / 2)) continue;
          const float terrain = heuristic ? 1.f + float(p_.terrain_weight) * std::max<int>(0, map.at(ni, nj)) / 100.f : 1.f;
          const float nd = dc + float(map.res * std::hypot(di, dj)) * terrain;
          if (nd < d.at(ni, nj)) d.at(ni, nj) = nd, q.push({nd, nj * map.w + ni});
        }
    }
    return d;
  }

  VehicleParams v_;
  PlannerParams p_;
  ReedsShepp rs_;
};

// True if the footprint along `path` from `from`, within `horizon` metres, hits a lethal cell.
inline bool pathBlocked(const Path& path, size_t from, const CostGrid& g, const VehicleParams& v, double horizon, double margin) {
  double s = 0;
  for (size_t i = from; i < path.size(); ++i) {
    if (i > from) s += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (s > horizon) break;
    if (footprintCollides(g, path[i].x, path[i].y, path[i].yaw, v, margin)) return true;
  }
  return false;
}

}  // namespace iad::planning
