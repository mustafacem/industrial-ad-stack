// Path tracking for a heavy vehicle: trajectory segments (gear changes), reference sampling,
// delay-compensated linear MPC (lateral), pure-pursuit baseline, and a longitudinal PI with
// speed / acceleration / grade feed-forward. ROS-free.
#pragma once
#include <deque>
#include <vector>

#include "iad_control/common.hpp"
#include "iad_control/qp.hpp"

namespace iad::control {

struct RefPoint {
  double x = 0, y = 0, yaw = 0, v = 0, a = 0, kappa = 0, s = 0;
  int dir = 1;
};
using RefPath = std::vector<RefPoint>;

// Gear of every point. A point's velocity sign is authoritative when it is moving; stop points (v = 0,
// at cusps, stop lines and the goal) take the gear of the motion that reaches them, which the path
// geometry tells: the displacement to the next point, projected on the heading.
inline void inferDirections(RefPath& p) {
  const size_t n = p.size();
  if (n == 0) return;
  std::vector<int> geo(n, 1);
  for (size_t i = 0; i < n; ++i) {
    const size_t a = i + 1 < n ? i : i - 1, b = i + 1 < n ? i + 1 : i;
    if (a == b) continue;
    const double d = (p[b].x - p[a].x) * std::cos(p[a].yaw) + (p[b].y - p[a].y) * std::sin(p[a].yaw);
    geo[i] = std::abs(d) < 1e-4 && i > 0 ? geo[i - 1] : d < 0 ? -1 : 1;  // duplicate points keep the gear
  }
  for (size_t i = 0; i < n; ++i) {
    if (std::abs(p[i].v) > 1e-3) p[i].dir = p[i].v < 0 ? -1 : 1;
    else p[i].dir = i > 0 ? geo[i - 1] : geo[i];  // reached by the motion from the previous point
  }
  // arc length
  p[0].s = 0;
  for (size_t i = 1; i < n; ++i) p[i].s = p[i - 1].s + std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
}

struct State {
  double x = 0, y = 0, yaw = 0, v = 0, steer = 0, pitch = 0;
};

struct Errors {
  double lat = 0, head = 0;
  size_t idx = 0;
};

// Signed lateral error (left of the path positive) and heading error at the nearest point in [b, e].
inline Errors trackingErrors(const RefPath& p, size_t b, size_t e, double x, double y, double yaw, size_t hint) {
  Errors out;
  double best = 1e18;
  const size_t lo = std::max(b, hint > 30 ? hint - 30 : 0), hi = std::min(e, hint + 60);
  for (size_t i = lo; i <= hi; ++i) {
    const double d = std::hypot(x - p[i].x, y - p[i].y);
    if (d < best) best = d, out.idx = i;
  }
  const auto& r = p[out.idx];
  out.lat = -(x - r.x) * std::sin(r.yaw) + (y - r.y) * std::cos(r.yaw);
  out.head = wrapAngle(yaw - r.yaw);
  return out;
}

struct MpcParams {
  int N = 30;
  double dt = 0.1;
  double q_lat = 1.0, q_head = 3.0, q_lat_T = 4.0, q_head_T = 12.0;
  double r_u = 0.1, r_du = 25.0;
  bool delay_comp = true;
};

class LateralMpc {
 public:
  LateralMpc(const VehicleParams& v, const MpcParams& p) : v_(v), p_(p) {}

  // e0 = (lat, head, steer) at the time the command will act. vs/ks: speed and curvature along the horizon.
  // Returns the steering command; `pred` receives the predicted (lat, head) errors.
  double solve(const Eigen::Vector3d& e0, const std::vector<double>& vs, const std::vector<double>& ks, double u_prev,
               std::vector<Eigen::Vector3d>* pred, int* iters) {
    const int N = p_.N, nx = 3;
    const double L = v_.wheelbase, tau = v_.steer_tau, dt = p_.dt;
    std::vector<Eigen::Matrix3d> A(static_cast<size_t>(N));
    std::vector<Eigen::Vector3d> B(static_cast<size_t>(N)), Wk(static_cast<size_t>(N));
    Eigen::VectorXd dref(N);
    for (int k = 0; k < N; ++k) {
      const double v = vs[size_t(k)], sgn = v >= 0 ? 1.0 : -1.0;
      const double dr = std::atan(L * ks[size_t(k)] * sgn), c2 = std::cos(dr) * std::cos(dr);
      dref[k] = std::clamp(dr, -v_.max_steer, v_.max_steer);
      A[size_t(k)] << 1, v * dt, 0, 0, 1, v * dt / (L * c2), 0, 0, 1 - dt / tau;
      B[size_t(k)] << 0, 0, dt / tau;
      Wk[size_t(k)] << 0, -v * dt * dr / (L * c2), 0;
    }
    // Condensed prediction X = Phi e0 + Gamma U + Omega, stacked over k = 1..N.
    Eigen::MatrixXd Phi(N * nx, nx), Gam = Eigen::MatrixXd::Zero(N * nx, N);
    Eigen::VectorXd Om(N * nx);
    Eigen::Matrix3d Ak = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ok = Eigen::Vector3d::Zero();
    for (int k = 0; k < N; ++k) {
      Ak = A[size_t(k)] * Ak;
      ok = A[size_t(k)] * ok + Wk[size_t(k)];
      Phi.block(k * nx, 0, nx, nx) = Ak;
      Om.segment(k * nx, nx) = ok;
      for (int j = 0; j <= k; ++j) {
        Eigen::Vector3d g = B[size_t(j)];
        for (int m = j + 1; m <= k; ++m) g = A[size_t(m)] * g;
        Gam.block(k * nx, j, nx, 1) = g;
      }
    }
    Eigen::VectorXd Qd(N * nx);
    for (int k = 0; k < N; ++k) {
      const bool T = k == N - 1;
      Qd.segment(k * nx, nx) << (T ? p_.q_lat_T : p_.q_lat), (T ? p_.q_head_T : p_.q_head), 0.0;
    }
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(N, N);
    for (int k = 1; k < N; ++k) D(k, k - 1) = -1;
    Eigen::VectorXd c = Eigen::VectorXd::Zero(N);
    c[0] = u_prev;
    const Eigen::MatrixXd GtQ = Gam.transpose() * Qd.asDiagonal();
    const Eigen::MatrixXd P = 2 * (GtQ * Gam + p_.r_u * Eigen::MatrixXd::Identity(N, N) + p_.r_du * D.transpose() * D);
    const Eigen::VectorXd q = 2 * (GtQ * (Phi * e0 + Om) - p_.r_u * dref - p_.r_du * D.transpose() * c);
    Eigen::MatrixXd Ac(2 * N, N);
    Ac << Eigen::MatrixXd::Identity(N, N), D;
    Eigen::VectorXd lo(2 * N), hi(2 * N);
    const double du = v_.max_steer_rate * dt * 1.2;
    lo << Eigen::VectorXd::Constant(N, -v_.max_steer), Eigen::VectorXd::Constant(N, -du) + c;
    hi << Eigen::VectorXd::Constant(N, v_.max_steer), Eigen::VectorXd::Constant(N, du) + c;
    Eigen::VectorXd warm;
    if (warm_.size() == N) {
      warm = warm_;
      for (int k = 0; k + 1 < N; ++k) warm[k] = warm_[k + 1];
    }
    const QpResult r = solveQp(P, q, Ac, lo, hi, warm.size() ? &warm : nullptr, 300);
    warm_ = r.x;
    if (iters) *iters = r.iterations;
    if (pred) {
      const Eigen::VectorXd X = Phi * e0 + Gam * r.x + Om;
      pred->clear();
      for (int k = 0; k < N; ++k) pred->push_back(X.segment(k * nx, nx));
    }
    return std::clamp(r.x[0], -v_.max_steer, v_.max_steer);
  }

 private:
  VehicleParams v_;
  MpcParams p_;
  Eigen::VectorXd warm_;
};

// Kinematic bicycle with first-order steering lag: replays the commands still in the actuator pipe.
inline State predictState(State s, const std::deque<double>& steer_cmds, double dt, const VehicleParams& v) {
  for (double u : steer_cmds) {
    s.steer += (u - s.steer) * std::min(1.0, dt / v.steer_tau);
    s.x += s.v * std::cos(s.yaw) * dt, s.y += s.v * std::sin(s.yaw) * dt;
    s.yaw = wrapAngle(s.yaw + s.v * std::tan(s.steer) / v.wheelbase * dt);
  }
  return s;
}

// Pure pursuit on the rear axle: the arc through the axle tangent to the heading reaching the lookahead
// point. The same arc geometry holds in reverse, so one formula serves both gears.
inline double purePursuit(const RefPath& p, size_t b, size_t e, size_t near, const State& s, double ld, const VehicleParams& v,
                          double* tx = nullptr, double* ty = nullptr) {
  double gx = p[e].x, gy = p[e].y;
  for (size_t i = near; i <= e; ++i)
    if (std::hypot(p[i].x - s.x, p[i].y - s.y) >= ld) {
      gx = p[i].x, gy = p[i].y;
      break;
    }
  if (std::hypot(gx - s.x, gy - s.y) < ld) {  // past the segment end: extend along the final heading
    const double ext = ld - std::hypot(gx - s.x, gy - s.y);
    gx += p[e].dir * ext * std::cos(p[e].yaw), gy += p[e].dir * ext * std::sin(p[e].yaw);
  }
  if (tx) *tx = gx, *ty = gy;
  const double c = std::cos(s.yaw), sn = std::sin(s.yaw), dx = gx - s.x, dy = gy - s.y;
  const double xl = c * dx + sn * dy, yl = -sn * dx + c * dy;
  return std::clamp(std::atan(v.wheelbase * 2.0 * yl / std::max(1e-6, xl * xl + yl * yl)), -v.max_steer, v.max_steer);
}

}  // namespace iad::control
