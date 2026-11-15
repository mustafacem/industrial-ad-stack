// Obstacle clustering (grid connected components), minimum-area oriented boxes, size-based classes and a
// greedy alpha-beta tracker. ROS-free.
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace iad::perception {

struct Obb {
  Eigen::Vector2d c = Eigen::Vector2d::Zero();
  double yaw = 0, length = 0, width = 0, z0 = 0, z1 = 0;
  int n = 0;
  std::string cls = "obstacle";
};

inline std::string classify(const Obb& b) {
  const double h = b.z1 - b.z0, L = std::max(b.length, b.width), W = std::min(b.length, b.width);
  if (L > 7.0 && W < 4.0 && h < 2.2) return "berm";  // long, low, thin: safety berm / windrow
  if (L < 1.3 && h > 1.0 && h < 2.3) return "person";
  if (L >= 3.0 && L <= 8.0 && W >= 1.2 && W <= 3.2 && h < 3.5) return "vehicle";
  if (L > 7.0 || h > 3.0) return "machine";
  return "obstacle";
}

// Minimum-area rectangle of 2D points via convex hull + edge-aligned bounding boxes.
inline Obb minAreaRect(std::vector<Eigen::Vector2d> pts) {
  Obb best;
  if (pts.empty()) return best;
  std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
  auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<Eigen::Vector2d> h(2 * pts.size());
  size_t k = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
    h[k++] = pts[i];
  }
  for (size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && cross(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
    h[k++] = pts[i];
  }
  h.resize(k > 1 ? k - 1 : k);
  double best_area = 1e18;
  for (size_t i = 0; i < std::max<size_t>(1, h.size()); ++i) {
    const Eigen::Vector2d e = h.size() > 1 ? (h[(i + 1) % h.size()] - h[i]) : Eigen::Vector2d(1, 0);
    const double yaw = std::atan2(e.y(), e.x()), c = std::cos(yaw), s = std::sin(yaw);
    double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18;
    for (const auto& p : h) {
      const double u = c * p.x() + s * p.y(), v = -s * p.x() + c * p.y();
      x0 = std::min(x0, u), x1 = std::max(x1, u), y0 = std::min(y0, v), y1 = std::max(y1, v);
    }
    const double area = (x1 - x0) * (y1 - y0);
    if (area < best_area) {
      best_area = area;
      const double uc = 0.5 * (x0 + x1), vc = 0.5 * (y0 + y1);
      best.c = Eigen::Vector2d(c * uc - s * vc, s * uc + c * vc);
      best.yaw = yaw, best.length = std::max(0.2, x1 - x0), best.width = std::max(0.2, y1 - y0);
    }
  }
  if (best.width > best.length) std::swap(best.length, best.width), best.yaw += M_PI / 2;
  return best;
}

// Connected components on an occupancy grid of the obstacle points (8-neighbourhood).
// `weak` (optional, per point): clusters with more than `max_weak_fraction` weak echoes are dropped as dust.
// `in_valid` (optional, size of P) is set to 1 for points that belong to a cluster of >= min_pts points:
// residual dust is a sparse scatter of tiny clusters, real obstacles are dense.
inline std::vector<Obb> clusterObstacles(const std::vector<Eigen::Vector3f>& P, const std::vector<int>& idx,
                                         const std::vector<float>& ground_h, double cell, int min_pts,
                                         std::vector<uint8_t>* in_valid = nullptr,
                                         const std::vector<uint8_t>* weak = nullptr, double max_weak_fraction = 1.0) {
  if (in_valid) in_valid->assign(P.size(), 0);
  std::unordered_map<int64_t, std::vector<int>> cells;
  auto key = [](int i, int j) { return (int64_t(i) << 32) ^ int64_t(uint32_t(j)); };
  for (int i : idx) {
    const auto& p = P[size_t(i)];
    cells[key(int(std::floor(p.x() / cell)), int(std::floor(p.y() / cell)))].push_back(i);
  }
  std::unordered_map<int64_t, int> label;
  std::vector<Obb> out;
  std::vector<int64_t> stack;
  for (const auto& kv : cells) {
    if (label.count(kv.first)) continue;
    std::vector<int> members;
    stack.assign(1, kv.first);
    label[kv.first] = int(out.size());
    while (!stack.empty()) {
      const int64_t c = stack.back();
      stack.pop_back();
      const auto& v = cells[c];
      members.insert(members.end(), v.begin(), v.end());
      const int ci = int(c >> 32), cj = int(int32_t(c & 0xffffffff));
      for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
          const int64_t nk = key(ci + di, cj + dj);
          if (cells.count(nk) && !label.count(nk)) label[nk] = int(out.size()), stack.push_back(nk);
        }
    }
    if (int(members.size()) < min_pts) continue;
    if (weak && max_weak_fraction < 1.0) {  // a cluster of mostly weak echoes is residual dust, not an object
      size_t nw = 0;
      for (int i : members) nw += (*weak)[size_t(i)];
      if (double(nw) > max_weak_fraction * double(members.size())) continue;
    }
    if (in_valid)
      for (int i : members) (*in_valid)[size_t(i)] = 1;
    std::vector<Eigen::Vector2d> xy;
    xy.reserve(members.size());
    double z0 = 1e9, z1 = -1e9;
    for (int i : members) {
      const auto& p = P[size_t(i)];
      xy.emplace_back(p.x(), p.y());
      z0 = std::min(z0, double(p.z() - ground_h[size_t(i)])), z1 = std::max(z1, double(p.z()));
    }
    Obb b = minAreaRect(xy);
    b.z0 = z0, b.z1 = z1, b.n = int(members.size());
    b.cls = classify(b);
    out.push_back(b);
  }
  return out;
}

struct Track {
  int id = 0, hits = 0, misses = 0;
  Eigen::Vector2d p = Eigen::Vector2d::Zero(), v = Eigen::Vector2d::Zero();
  Obb box;
  double last_t = 0;
  bool confirmed() const { return hits >= 3; }
  double speed() const { return v.norm(); }
};

// Greedy nearest-neighbour association + alpha-beta filter, in the map frame.
class Tracker {
 public:
  void update(const std::vector<Obb>& dets, double t) {
    std::vector<bool> used(dets.size(), false);
    for (auto& tr : tracks_) {
      const double dt = std::max(1e-3, t - tr.last_t);
      const Eigen::Vector2d pred = tr.p + tr.v * dt;
      int best = -1;
      double bd = 3.0 + 0.5 * tr.speed() * dt;
      for (size_t k = 0; k < dets.size(); ++k) {
        const double d = (dets[k].c - pred).norm();
        if (!used[k] && d < bd) bd = d, best = int(k);
      }
      if (best < 0) {
        ++tr.misses;
        continue;
      }
      used[size_t(best)] = true;
      const Eigen::Vector2d r = dets[size_t(best)].c - pred;
      tr.p = pred + 0.5 * r;
      tr.v = 0.8 * tr.v + 0.2 * (tr.v + r / dt);
      if (tr.v.norm() > 25) tr.v.setZero();
      tr.box = dets[size_t(best)], tr.last_t = t, ++tr.hits, tr.misses = 0;
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [](const Track& tr) { return tr.misses > 5; }),
                  tracks_.end());
    for (size_t k = 0; k < dets.size(); ++k)
      if (!used[k]) {
        Track tr;
        tr.id = next_id_++, tr.p = dets[k].c, tr.box = dets[k], tr.last_t = t, tr.hits = 1;
        tracks_.push_back(tr);
      }
  }
  const std::vector<Track>& tracks() const { return tracks_; }

 private:
  std::vector<Track> tracks_;
  int next_id_ = 1;
};

}  // namespace iad::perception
