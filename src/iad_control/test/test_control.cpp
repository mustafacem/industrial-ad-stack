#include <gtest/gtest.h>

#include <random>

#include "iad_control/qp.hpp"
#include "iad_control/tracking.hpp"

using namespace iad::control;

TEST(Qp, MatchesUnconstrainedAndRespectsBounds) {
  std::mt19937 rng(1);
  std::normal_distribution<double> N(0, 1);
  Eigen::MatrixXd M(6, 6);
  for (int i = 0; i < 36; ++i) M.data()[i] = N(rng);
  const Eigen::MatrixXd P = M * M.transpose() + Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd q(6);
  for (int i = 0; i < 6; ++i) q[i] = N(rng);
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  const auto free = solveQp(P, q, A, Eigen::VectorXd::Constant(6, -1e9), Eigen::VectorXd::Constant(6, 1e9));
  EXPECT_LT((free.x - P.ldlt().solve(-q)).norm(), 1e-3);
  const auto box = solveQp(P, q, A, Eigen::VectorXd::Constant(6, -0.1), Eigen::VectorXd::Constant(6, 0.1));
  EXPECT_LE(box.x.cwiseAbs().maxCoeff(), 0.1 + 1e-4);
}

// Closed loop on a straight road with a 1.5 m initial offset, 0.4 s dead time and steering lag:
// the delay-compensated MPC must converge without a large overshoot.
TEST(Mpc, ConvergesWithDeadTime) {
  const VehicleParams v = VehicleParams{};
  LateralMpc mpc(v, MpcParams{});
  RefPath path;
  for (int i = 0; i < 600; ++i) path.push_back({double(i) * 0.5, 0.0, 0.0, 6.0, 0.0, 0.0, double(i) * 0.5, 1});
  State s;
  s.y = 1.5, s.v = 6.0;
  std::deque<double> pipe(size_t(v.delay / 0.05), 0.0), hist = pipe;
  double steer = 0, u_prev = 0, min_y = 1e9;
  for (int k = 0; k < 300; ++k) {  // 15 s at 20 Hz
    State sp = predictState(s, hist, 0.05, v);
    const Errors e = trackingErrors(path, 0, path.size() - 1, sp.x, sp.y, sp.yaw, size_t(sp.x / 0.5));
    std::vector<double> vs(30, 6.0), ks(30, 0.0);
    const double u = mpc.solve(Eigen::Vector3d(e.lat, e.head, sp.steer), vs, ks, u_prev, nullptr, nullptr);
    u_prev = u, hist.push_back(u), hist.pop_front(), pipe.push_back(u);
    const double ua = pipe.front();
    pipe.pop_front();
    for (int i = 0; i < 5; ++i) {  // plant at 100 Hz: dead time, lag, rate limit
      steer += std::clamp((ua - steer) * 0.01 / v.steer_tau, -v.max_steer_rate * 0.01, v.max_steer_rate * 0.01);
      s.x += s.v * std::cos(s.yaw) * 0.01, s.y += s.v * std::sin(s.yaw) * 0.01, s.yaw += s.v * std::tan(steer) / v.wheelbase * 0.01;
    }
    s.steer = steer;
    min_y = std::min(min_y, s.y);
  }
  EXPECT_LT(std::abs(s.y), 0.05);
  EXPECT_GT(min_y, -0.3);  // overshoot below 20 %
}
