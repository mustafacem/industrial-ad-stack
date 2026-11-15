// Ground segmentation for uneven terrain. ROS-free.
//
// segmentGroundCzm: concentric-zone model (in the spirit of Patchwork): polar bins, a PCA plane per bin
// from the lowest seeds, accepted when (a) upright enough to drive on, and (b) it meets the accepted plane
// radially inside it at their shared boundary (terrain is continuous on any grade; the flat top of a
// cabin, truck bed or boulder is not).
// Runs in a gravity-aligned frame so a pitched vehicle on a ramp doesn't tilt the ground.
//
// segmentGroundRansac: the baseline, one global plane in the sensor frame.
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace iad::perception {

struct GroundResult {
  std::vector<uint8_t> ground;  // 1 = ground
  std::vector<float> height;    // signed height above the local ground plane [m]
  int bins = 0, bins_accepted = 0;
};

struct CzmParams {
  std::vector<double> edges{2.5, 12.0, 26.0, 46.0, 80.0};
  std::vector<int> rings{2, 4, 4, 4};
  std::vector<int> sectors{16, 32, 54, 32};
  int seed_n = 20;
  double seed_dz = 0.5;
  double dist_th = 0.18;     // inlier band for the plane fit
  double obstacle_h = 0.30;  // above the plane by more than this -> obstacle
  int iters = 3, min_pts = 10;
  double min_upright = 0.88;  // cos(28 deg)
  double boundary_tol = 0.5, boundary_tol_rel = 0.02;  // allowed plane mismatch at the shared boundary: a + b*r
  double gap_grade = 0.15;  // + this grade over any unobserved gap since the reference plane was last seen
  double min_radial_extent = 1.5;  // a bin seen only as a thin arc (e.g. the lowest ring) cannot tilt its plane
};

namespace detail {
struct Plane {
  Eigen::Vector3d n{0, 0, 1};
  double d = 0;        // n.p + d = 0
  double r_valid = 3;  // radius up to which this plane was actually observed
  double heightAt(double x, double y) const { return -(n.x() * x + n.y() * y + d) / n.z(); }
  double dist(const Eigen::Vector3f& p) const { return n.dot(p.cast<double>()) + d; }
};

inline bool fitPlane(const std::vector<Eigen::Vector3f>& P, const std::vector<int>& idx, Plane& pl, double& flat) {
  if (idx.size() < 3) return false;
  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  for (int i : idx) mu += P[size_t(i)].cast<double>();
  mu /= double(idx.size());
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (int i : idx) {
    const Eigen::Vector3d d = P[size_t(i)].cast<double>() - mu;
    C += d * d.transpose();
  }
  C /= double(idx.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  pl.n = es.eigenvectors().col(0);
  if (pl.n.z() < 0) pl.n = -pl.n;
  pl.d = -pl.n.dot(mu);
  flat = es.eigenvalues()(0);
  return pl.n.z() > 1e-3;
}
}  // namespace detail

// `wheel_normal`: the vehicle's ground plane (its z axis) in the level frame. The wheels sit on the
// terrain, so near the vehicle the ground continues along that plane (e.g. up a ramp).
inline GroundResult segmentGroundCzm(const std::vector<Eigen::Vector3f>& P, const std::vector<uint8_t>& ignore,
                                     const CzmParams& prm, const Eigen::Vector3d& wheel_normal = Eigen::Vector3d::UnitZ()) {
  using detail::Plane;
  GroundResult out;
  const size_t n = P.size();
  out.ground.assign(n, 0);
  out.height.assign(n, 0.f);
  const int Z = int(prm.rings.size());
  std::vector<int> ring_off(size_t(Z) + 1, 0);
  for (int z = 0; z < Z; ++z) ring_off[size_t(z) + 1] = ring_off[size_t(z)] + prm.rings[size_t(z)];
  const int G = ring_off[size_t(Z)];
  auto zoneOf = [&](int g) {
    int z = 0;
    while (g >= ring_off[size_t(z) + 1]) ++z;
    return z;
  };
  auto ringInner = [&](int g) {  // inner edge radius of global ring g
    const int z = zoneOf(g), k = g - ring_off[size_t(z)];
    const double e0 = prm.edges[size_t(z)], e1 = prm.edges[size_t(z) + 1];
    return e0 + k * (e1 - e0) / prm.rings[size_t(z)];
  };
  std::vector<std::vector<std::vector<int>>> bins(static_cast<size_t>(G));
  for (int g = 0; g < G; ++g) bins[size_t(g)].resize(size_t(prm.sectors[size_t(zoneOf(g))]));
  std::vector<int> unbinned;
  for (size_t i = 0; i < n; ++i) {
    if (!ignore.empty() && ignore[i]) continue;
    const double r = std::hypot(P[i].x(), P[i].y());
    if (r < prm.edges.front() || r >= prm.edges.back()) {
      unbinned.push_back(int(i));
      continue;
    }
    int z = 0;
    while (r >= prm.edges[size_t(z) + 1]) ++z;
    const double e0 = prm.edges[size_t(z)], e1 = prm.edges[size_t(z) + 1];
    const int k = std::min(prm.rings[size_t(z)] - 1, int((r - e0) / (e1 - e0) * prm.rings[size_t(z)]));
    const int S = prm.sectors[size_t(z)];
    const int s = std::min(S - 1, int((std::atan2(P[i].y(), P[i].x()) + M_PI) / (2 * M_PI) * S));
    bins[size_t(ring_off[size_t(z)] + k)][size_t(s)].push_back(int(i));
  }
  // Reference plane per bin: its own if accepted, else inherited from the bin radially inside.
  std::vector<std::vector<Plane>> ref(static_cast<size_t>(G));
  Plane base;  // the plane under the wheels, through the level-frame origin (base_link is on the ground)
  base.n = wheel_normal.normalized();
  for (int g = 0; g < G; ++g) {
    const int S = int(bins[size_t(g)].size());
    ref[size_t(g)].assign(size_t(S), base);
    for (int s = 0; s < S; ++s) {
      const double th = -M_PI + (s + 0.5) * 2 * M_PI / S, rb = ringInner(g);
      const double bx = rb * std::cos(th), by = rb * std::sin(th);  // shared boundary with the inner bin
      Plane inner = base;
      if (g > 0) {
        const int Si = int(bins[size_t(g) - 1].size());
        inner = ref[size_t(g) - 1][size_t(std::min(Si - 1, int((th + M_PI) / (2 * M_PI) * Si)))];
      }
      auto& idx = bins[size_t(g)][size_t(s)];
      ++out.bins;
      Plane pl;
      bool accepted = false;
      if (int(idx.size()) >= prm.min_pts) {
        std::sort(idx.begin(), idx.end(), [&](int a, int b) { return P[size_t(a)].z() < P[size_t(b)].z(); });
        double lpr = 0;
        const int ns = std::min<int>(prm.seed_n, int(idx.size()));
        for (int k = 0; k < ns; ++k) lpr += P[size_t(idx[size_t(k)])].z();
        lpr /= ns;
        std::vector<int> g_idx;
        for (int i : idx)
          if (P[size_t(i)].z() < lpr + prm.seed_dz) g_idx.push_back(i);
        double flat = 0;
        bool ok = true;
        for (int it = 0; it < prm.iters && ok; ++it) {
          ok = detail::fitPlane(P, g_idx, pl, flat);
          if (!ok) break;
          g_idx.clear();
          for (int i : idx)
            if (std::abs(pl.dist(P[size_t(i)])) < prm.dist_th) g_idx.push_back(i);
          ok = g_idx.size() >= 5;
        }
        if (ok) {
          // Thin arc (the first ring that reaches the ground, or a bin clipped by an obstacle): the radial
          // tilt is unobservable, so keep the inner plane's normal and fit only the height through the inliers.
          double rmin = 1e9, rmax = -1e9;
          Eigen::Vector3d c = Eigen::Vector3d::Zero();
          for (int i : g_idx) {
            const auto& q = P[size_t(i)];
            const double r = std::hypot(q.x(), q.y());
            rmin = std::min(rmin, r), rmax = std::max(rmax, r), c += q.cast<double>();
          }
          if (rmax - rmin < prm.min_radial_extent && !g_idx.empty()) {
            c /= double(g_idx.size());
            pl.n = inner.n, pl.d = -inner.n.dot(c);
          }
          const double jump = std::abs(pl.heightAt(bx, by) - inner.heightAt(bx, by));
          const double allowed = prm.boundary_tol + prm.boundary_tol_rel * rb + prm.gap_grade * std::max(0.0, rb - inner.r_valid);
          accepted = pl.n.z() >= prm.min_upright && jump <= allowed;
        }
      }
      const Plane& use = accepted ? pl : inner;
      if (accepted) pl.r_valid = ringInner(g + 1 < G ? g + 1 : g) + (g + 1 < G ? 0.0 : 10.0), ref[size_t(g)][size_t(s)] = pl, ++out.bins_accepted;
      else ref[size_t(g)][size_t(s)] = inner;
      for (int i : idx) {
        const double h = use.dist(P[size_t(i)]);
        out.height[size_t(i)] = float(h);
        out.ground[size_t(i)] = h < prm.obstacle_h;
      }
    }
  }
  for (int i : unbinned) {  // very close (ego surroundings) or beyond range: plain height
    out.height[size_t(i)] = P[size_t(i)].z();
    out.ground[size_t(i)] = P[size_t(i)].z() < prm.obstacle_h;
  }
  return out;
}

inline GroundResult segmentGroundRansac(const std::vector<Eigen::Vector3f>& P, const std::vector<uint8_t>& ignore,
                                        double th, int iters, uint32_t seed) {
  using detail::Plane;
  GroundResult out;
  const size_t n = P.size();
  out.ground.assign(n, 0);
  out.height.assign(n, 0.f);
  std::vector<int> idx;
  for (size_t i = 0; i < n; ++i)
    if (ignore.empty() || !ignore[i]) idx.push_back(int(i));
  if (idx.size() < 3) return out;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<size_t> U(0, idx.size() - 1);
  Plane best;
  size_t best_in = 0;
  for (int it = 0; it < iters; ++it) {
    const Eigen::Vector3d a = P[size_t(idx[U(rng)])].cast<double>(), b = P[size_t(idx[U(rng)])].cast<double>(),
                          c = P[size_t(idx[U(rng)])].cast<double>();
    Plane pl;
    pl.n = (b - a).cross(c - a);
    if (pl.n.norm() < 1e-6) continue;
    pl.n.normalize();
    if (pl.n.z() < 0) pl.n = -pl.n;
    if (pl.n.z() < 0.9) continue;  // a "ground" plane must be roughly horizontal
    pl.d = -pl.n.dot(a);
    size_t in = 0;
    for (size_t k = 0; k < idx.size(); k += 4) in += std::abs(pl.dist(P[size_t(idx[k])])) < th;
    if (in > best_in) best_in = in, best = pl;
  }
  for (int i : idx) {
    const double h = best.dist(P[size_t(i)]);
    out.height[size_t(i)] = float(h);
    out.ground[size_t(i)] = std::abs(h) < th;
  }
  return out;
}

}  // namespace iad::perception
