// Rolling log-odds occupancy grid in the map frame with free-space ray tracing. ROS-free.
// Transient returns (residual dust, a bird) get one hit and are then traced free again by the next
// sweeps; real obstacles accumulate. Each cell gets at most one update per scan.
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "iad_perception/types.hpp"

namespace iad::perception {

class LogOddsGrid {
 public:
  struct Params {
    double size = 140.0, res = 0.5;
    // A solid obstacle is hit on (nearly) every sweep, a dust plume cell only now and then: misses weigh almost
    // as much as hits, so intermittent returns never build up to "occupied".
    float l_hit = 0.85f, l_miss = -0.7f, l_min = -2.0f, l_max = 3.5f, l_occ = 1.6f, l_free = -0.4f;
  };
  explicit LogOddsGrid(const Params& p) : p_(p), n_(int(p.size / p.res)) {
    L_.assign(size_t(n_) * size_t(n_), 0.f);
    hit_stamp_.assign(L_.size(), 0);
    miss_stamp_.assign(L_.size(), 0);
  }

  // Keep the vehicle near the centre: shift the window in whole cells when it drifts > 1/4 size.
  void recenter(double x, double y) {
    if (!init_) {
      ox_ = std::floor((x - p_.size / 2) / p_.res) * p_.res, oy_ = std::floor((y - p_.size / 2) / p_.res) * p_.res;
      init_ = true;
      return;
    }
    const double cx = ox_ + p_.size / 2, cy = oy_ + p_.size / 2;
    if (std::abs(x - cx) < p_.size / 4 && std::abs(y - cy) < p_.size / 4) return;
    const int di = int(std::lround((x - cx) / p_.res)), dj = int(std::lround((y - cy) / p_.res));
    std::vector<float> L2(L_.size(), 0.f);
    for (int j = 0; j < n_; ++j)
      for (int i = 0; i < n_; ++i) {
        const int si = i + di, sj = j + dj;
        if (si >= 0 && sj >= 0 && si < n_ && sj < n_) L2[idx(i, j)] = L_[idx(si, sj)];
      }
    L_.swap(L2);
    ox_ += di * p_.res, oy_ += dj * p_.res;
  }

  void integrate(const Eigen::Vector2d& o, const std::vector<Eigen::Vector2d>& hits,
                 const std::vector<Eigen::Vector2d>& free_ends) {
    ++scan_;
    int oi, oj;
    if (!toCell(o, oi, oj)) return;
    auto trace = [&](const Eigen::Vector2d& e, bool hit) {
      int ei, ej;
      const bool inside = toCell(e, ei, ej);
      if (!inside) {  // clip the ray to the window
        const Eigen::Vector2d d = e - o;
        const double lo_x = ox_ + 1e-3, hi_x = ox_ + n_ * p_.res - 1e-3, lo_y = oy_ + 1e-3, hi_y = oy_ + n_ * p_.res - 1e-3;
        double t = 1.0;
        if (d.x() > 1e-9) t = std::min(t, (hi_x - o.x()) / d.x());
        else if (d.x() < -1e-9) t = std::min(t, (lo_x - o.x()) / d.x());
        if (d.y() > 1e-9) t = std::min(t, (hi_y - o.y()) / d.y());
        else if (d.y() < -1e-9) t = std::min(t, (lo_y - o.y()) / d.y());
        toCell(o + std::max(0.0, t) * d, ei, ej);
      }
      int x = oi, y = oj;
      const int dx = std::abs(ei - oi), dy = -std::abs(ej - oj), sx = oi < ei ? 1 : -1, sy = oj < ej ? 1 : -1;
      int err = dx + dy;
      while (!(x == ei && y == ej)) {
        miss_stamp_[idx(x, y)] = scan_;
        const int e2 = 2 * err;
        if (e2 >= dy) err += dy, x += sx;
        if (e2 <= dx) err += dx, y += sy;
      }
      if (hit && inside) hit_stamp_[idx(ei, ej)] = scan_;
      else miss_stamp_[idx(ei, ej)] = scan_;
    };
    for (const auto& e : free_ends) trace(e, false);
    for (const auto& h : hits) trace(h, true);
    for (size_t k = 0; k < L_.size(); ++k) {
      if (hit_stamp_[k] == scan_) L_[k] = std::min(p_.l_max, L_[k] + p_.l_hit);
      else if (miss_stamp_[k] == scan_) L_[k] = std::max(p_.l_min, L_[k] + p_.l_miss);
    }
  }

  bool occupied(size_t k) const { return L_[k] > p_.l_occ; }
  bool lastHit(size_t k) const { return hit_stamp_[k] == scan_; }
  size_t size() const { return L_.size(); }
  int n() const { return n_; }
  double ox() const { return ox_; }
  double oy() const { return oy_; }
  double res() const { return p_.res; }
  bool toCell(const Eigen::Vector2d& p, int& i, int& j) const {
    i = int(std::floor((p.x() - ox_) / p_.res)), j = int(std::floor((p.y() - oy_) / p_.res));
    return i >= 0 && j >= 0 && i < n_ && j < n_;
  }
  size_t idx(int i, int j) const { return size_t(j) * size_t(n_) + size_t(i); }

  CostGrid toCostGrid() const {
    CostGrid g;
    g.w = g.h = n_, g.res = p_.res, g.ox = ox_, g.oy = oy_;
    g.data.assign(L_.size(), kUnknown);
    for (size_t k = 0; k < L_.size(); ++k)
      g.data[k] = L_[k] > p_.l_occ ? kLethal : (L_[k] < p_.l_free ? int8_t(0) : kUnknown);
    return g;
  }

 private:
  Params p_;
  int n_;
  double ox_ = 0, oy_ = 0;
  bool init_ = false;
  uint32_t scan_ = 0;
  std::vector<float> L_;
  std::vector<uint32_t> hit_stamp_, miss_stamp_;
};

}  // namespace iad::perception
