#include <gtest/gtest.h>

#include <random>

#include "iad_localization/ekf.hpp"
#include "iad_localization/ndt.hpp"

using namespace iad::localization;

// Synthetic yard: ground + container-like walls, surfaces sampled at random (map and scan use different
// samples, like a real survey vs a real scan).
static std::vector<Eigen::Vector3d> yard(bool corridor_only, uint32_t seed, double density) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(0, 1);
  std::vector<Eigen::Vector3d> p;
  for (int k = 0; k < int(6400 * density * 2); ++k) p.emplace_back(-40 + 80 * U(rng), -40 + 80 * U(rng), 0.02 * (U(rng) - 0.5));
  auto wall = [&](double x0, double y0, double x1, double y1, double h) {
    const double L = std::hypot(x1 - x0, y1 - y0);
    for (int k = 0; k < int(L * h * density); ++k) {
      const double s = U(rng), z = h * U(rng);
      p.emplace_back(x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, z);
    }
  };
  wall(-40, 8, 40, 8, 8), wall(-40, -8, 40, -8, 8);  // a corridor of stacks along x
  if (!corridor_only) wall(12, -8, 12, -20, 6), wall(-15, 8, -15, 20, 6), wall(25, 15, 30, 25, 4), wall(-30, -8, -30, -18, 5);
  return p;
}

TEST(Ndt, ConvergesFromOffset) {
  NdtPyramid map;
  map.build(yard(false, 1, 6.0), 2.0);
  const Eigen::Isometry3d T_true = makeIso(1.0, 0.5, 0.0, 0.0, 0.0, 0.2);
  std::vector<Eigen::Vector3d> src;
  for (const auto& p : yard(false, 2, 1.0)) src.push_back(T_true.inverse() * p);
  const Eigen::Isometry3d init = T_true * makeIso(0.9, -0.7, 0.1, 0.0, 0.0, deg2rad(3));
  const NdtResult r = map.align(src, init);
  const auto [rot, trans] = poseError(r.T, T_true);
  std::printf("converge: iters %d matched %d nvtl %.3f trans %.3f rot %.2f deg  T=(%.2f %.2f %.2f)\n", r.iterations, r.matched, r.nvtl, trans, rad2deg(rot), r.T.translation().x(), r.T.translation().y(), r.T.translation().z());
  EXPECT_LT(trans, 0.10);
  EXPECT_LT(rad2deg(rot), 0.3);
}

TEST(Ndt, CovarianceGrowsAlongUnconstrainedAxis) {
  NdtPyramid map;  // two parallel walls: sliding along the corridor (x) is unobservable
  map.build(yard(true, 1, 6.0), 2.0);
  std::vector<Eigen::Vector3d> src;
  for (const auto& p : yard(true, 2, 1.0))
    if (std::abs(p.x()) < 30) src.push_back(p);
  const NdtResult r = map.align(src, Eigen::Isometry3d::Identity());
  std::printf("corridor: iters %d matched %d nvtl %.3f T=(%.3f %.3f %.3f) Hdiag=", r.iterations, r.matched, r.nvtl, r.T.translation().x(), r.T.translation().y(), r.T.translation().z());
  for (int k = 0; k < 6; ++k) std::printf("%.1f ", r.H(k, k));
  std::printf(" cov xx %.5f yy %.5f\n", r.cov_raw(0, 0), r.cov_raw(1, 1));
  EXPECT_GT(r.cov_raw(0, 0), 20.0 * r.cov_raw(1, 1));
}

TEST(Ekf, GatingRejectsMultipathJump) {
  Ekf f;
  f.init(0.0, 0, 0, 0, 0.1, 0.01);
  for (int k = 1; k <= 100; ++k) f.predict(0.01 * k, 2.0, 0.0);
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.04;
  EXPECT_TRUE(f.updateGnss(1.0, {2.0, 0.0}, R, Eigen::Vector2d::Zero()));
  EXPECT_FALSE(f.updateGnss(1.0, {2.0, 9.0}, R, Eigen::Vector2d::Zero()));  // 9 m multipath jump
  EXPECT_NEAR(f.x()(1), 0.0, 0.1);
}

TEST(Ekf, DelayedMeasurementMatchesInOrder) {
  // Same measurement applied late (after 20 more inputs) must give the same state as applied on time.
  Ekf a, b;
  for (Ekf* f : {&a, &b}) f->init(0.0, 0, 0, 0.1, 0.5, 0.05);
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.01;
  for (int k = 1; k <= 200; ++k) {
    const double t = 0.01 * k, w = 0.05 * std::sin(t);
    a.predict(t, 3.0, w), b.predict(t, 3.0, w);
    if (k == 100) a.updateGnss(1.0, {3.2, 0.5}, R, Eigen::Vector2d::Zero());
    if (k == 120) b.updateGnss(1.0, {3.2, 0.5}, R, Eigen::Vector2d::Zero());
  }
  EXPECT_NEAR(a.x()(0), b.x()(0), 0.02);
  EXPECT_NEAR(a.x()(1), b.x()(1), 0.02);
}
