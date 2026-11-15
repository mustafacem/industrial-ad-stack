// Global initialisation by correlative scan matching. ROS-free.
//
// GNSS gives the position to a few metres but no heading, and NDT only converges from a few degrees /
// a couple of metres. So: project the map's *structure* (points well above the local ground) into a 2D
// likelihood field, then brute-force the scan's structure points over every heading (1 deg) and over the
// GNSS uncertainty window (0.5 m), and hand the best pose to NDT. Ground points are excluded on both
// sides: they fit every heading equally well.
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <vector>

namespace iad::localization {

class LikelihoodField {
 public:
  void build(const std::vector<Eigen::Vector3d>& map, double res = 0.5, double sigma = 0.5, double min_height = 0.8) {
    res_ = res;
    double x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
    for (const auto& p : map) x0 = std::min(x0, p.x()), y0 = std::min(y0, p.y()), x1 = std::max(x1, p.x()), y1 = std::max(y1, p.y());
    if (map.empty()) return;
    ox_ = x0 - 5, oy_ = y0 - 5, w_ = int((x1 - x0 + 10) / res) + 1, h_ = int((y1 - y0 + 10) / res) + 1;
    // lowest point per 2 m column = local ground
    const int k = std::max(1, int(std::lround(2.0 / res)));
    const int cw = w_ / k + 1, ch = h_ / k + 1;
    std::vector<double> ground(size_t(cw) * size_t(ch), std::numeric_limits<double>::max());
    for (const auto& p : map) {
      const int i = int((p.x() - ox_) / res) / k, j = int((p.y() - oy_) / res) / k;
      auto& g = ground[size_t(j) * size_t(cw) + size_t(i)];
      g = std::min(g, p.z());
    }
    const float inf = 1e9f;
    std::vector<float> d(size_t(w_) * size_t(h_), inf);
    n_structure_ = 0;
    for (const auto& p : map) {
      const int i = int((p.x() - ox_) / res), j = int((p.y() - oy_) / res);
      if (p.z() - ground[size_t(j / k) * size_t(cw) + size_t(i / k)] < min_height) continue;
      d[idx(i, j)] = 0.f, ++n_structure_;
    }
    // two-pass chamfer distance transform (in cells)
    const float a = 1.f, b = 1.4142f;
    for (int j = 0; j < h_; ++j)
      for (int i = 0; i < w_; ++i) {
        float& v = d[idx(i, j)];
        if (i > 0) v = std::min(v, d[idx(i - 1, j)] + a);
        if (j > 0) v = std::min(v, d[idx(i, j - 1)] + a);
        if (i > 0 && j > 0) v = std::min(v, d[idx(i - 1, j - 1)] + b);
        if (i + 1 < w_ && j > 0) v = std::min(v, d[idx(i + 1, j - 1)] + b);
      }
    for (int j = h_ - 1; j >= 0; --j)
      for (int i = w_ - 1; i >= 0; --i) {
        float& v = d[idx(i, j)];
        if (i + 1 < w_) v = std::min(v, d[idx(i + 1, j)] + a);
        if (j + 1 < h_) v = std::min(v, d[idx(i, j + 1)] + a);
        if (i + 1 < w_ && j + 1 < h_) v = std::min(v, d[idx(i + 1, j + 1)] + b);
        if (i > 0 && j + 1 < h_) v = std::min(v, d[idx(i - 1, j + 1)] + b);
      }
    f_.resize(d.size());
    const double s2 = 2 * sigma * sigma;
    for (size_t q = 0; q < d.size(); ++q) {
      const double m = d[q] * res;
      f_[q] = float(std::exp(-m * m / s2));
    }
  }

  struct Result {
    double x = 0, y = 0, yaw = 0, score = 0;  // score: mean field value of the scan structure points (0..1)
  };

  // `scan` = structure points in the base frame. (fx, fy) = GNSS antenna position, `lever` = antenna in the
  // base frame: for each heading the search window is centred on fix - R(yaw) * lever.
  Result search(const std::vector<Eigen::Vector2d>& scan, double fx, double fy, const Eigen::Vector2d& lever,
                double window = 4.0, double yaw_step = M_PI / 180.0) const {
    Result best;
    if (f_.empty() || scan.empty()) return best;
    const int nxy = int(std::ceil(window / res_));
    std::vector<Eigen::Vector2d> rot(scan.size());
    for (double yaw = -M_PI; yaw < M_PI; yaw += yaw_step) {
      const double c = std::cos(yaw), s = std::sin(yaw);
      for (size_t k = 0; k < scan.size(); ++k) rot[k] = {c * scan[k].x() - s * scan[k].y(), s * scan[k].x() + c * scan[k].y()};
      const double gx = fx - (c * lever.x() - s * lever.y()), gy = fy - (s * lever.x() + c * lever.y());
      for (int dj = -nxy; dj <= nxy; ++dj)
        for (int di = -nxy; di <= nxy; ++di) {
          const double tx = gx + di * res_, ty = gy + dj * res_;
          double sum = 0;
          for (const auto& p : rot) sum += at(p.x() + tx, p.y() + ty);
          const double sc = sum / double(rot.size());
          if (sc > best.score) best = {tx, ty, yaw, sc};
        }
    }
    return best;
  }

  size_t structurePoints() const { return n_structure_; }

 private:
  size_t idx(int i, int j) const { return size_t(j) * size_t(w_) + size_t(i); }
  float at(double x, double y) const {
    const int i = int((x - ox_) / res_), j = int((y - oy_) / res_);
    return i >= 0 && j >= 0 && i < w_ && j < h_ ? f_[idx(i, j)] : 0.f;
  }
  double res_ = 0.5, ox_ = 0, oy_ = 0;
  int w_ = 0, h_ = 0;
  size_t n_structure_ = 0;
  std::vector<float> f_;
};

}  // namespace iad::localization
