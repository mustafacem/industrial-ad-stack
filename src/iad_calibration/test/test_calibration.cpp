#include <gtest/gtest.h>

#include <random>

#include "iad_calibration/calibration.hpp"


using namespace iad::calibration;

// A corner LiDAR pitched down 10 deg, rolled 3 deg, 1.95 m up: the ground plane alone recovers roll,
// pitch and height exactly; yaw/x/y are untouched.
TEST(Calibration, GroundPlaneRecoversRollPitchHeight) {
  const Eigen::Isometry3d T_true = makeIso(10.0, 1.5, 1.95, deg2rad(3.0), deg2rad(10.0), deg2rad(45.0));
  std::mt19937 rng(2);
  std::uniform_real_distribution<double> U(-15, 15);
  std::normal_distribution<double> N(0, 0.01);
  std::vector<Eigen::Vector3d> pts;
  for (int k = 0; k < 3000; ++k) pts.push_back(T_true.inverse() * Eigen::Vector3d(10 + U(rng), 1.5 + U(rng), N(rng)));
  const Eigen::Isometry3d guess = makeIso(10.2, 1.4, 1.7, deg2rad(-1.0), deg2rad(13.0), deg2rad(47.0));
  const Eigen::Isometry3d T = applyGroundPlane(guess, fitGroundPlane(pts));
  const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(T.linear()));
  EXPECT_NEAR(rad2deg(rpy.x()), 3.0, 0.1);
  EXPECT_NEAR(rad2deg(rpy.y()), 10.0, 0.1);
  EXPECT_NEAR(T.translation().z(), 1.95, 0.01);
}

// Joint registration over keyframes recovers a 2 deg / 25 cm yaw-x-y error of the extrinsic.
TEST(Calibration, RegistrationRecoversYawXY) {
  std::mt19937 rng(4);
  std::uniform_real_distribution<double> U(0, 1);
  std::vector<Eigen::Vector3d> world;
  auto wall = [&](double x0, double y0, double x1, double y1, double h) {
    for (int k = 0; k < int(std::hypot(x1 - x0, y1 - y0) * h * 6); ++k) {
      const double s = U(rng);
      world.emplace_back(x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, h * U(rng));
    }
  };
  for (int k = 0; k < 40000; ++k) world.emplace_back(-20 + 140 * U(rng), -30 + 60 * U(rng), 0.0);
  wall(-20, 15, 120, 15, 8), wall(-20, -25, 120, -25, 6), wall(30, 15, 30, 5, 3), wall(70, -25, 70, -12, 4), wall(100, 15, 104, 6, 5);
  SurfelMap map;
  map.build(world, 1.0, 0.05, 5);
  const Eigen::Isometry3d T_true = makeIso(10.0, 1.5, 1.95, 0.0, deg2rad(10.0), deg2rad(45.0));
  std::vector<Keyframe> kfs;
  for (double x = 0; x < 100; x += 4) {
    Keyframe kf;
    kf.T_map_base = makeIso(x, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Eigen::Isometry3d T_ms = kf.T_map_base * T_true;
    for (const auto& p : world) {
      const Eigen::Vector3d ps = T_ms.inverse() * p;
      if (ps.norm() < 40 && ps.x() > 0 && U(rng) < 0.25) kf.pts.push_back(ps);  // forward-looking sensor
    }
    kfs.push_back(kf);
  }
  Eigen::Isometry3d T = makeIso(10.2, 1.35, 1.95, 0.0, deg2rad(10.0), deg2rad(47.0));
  for (int it = 0; it < 30; ++it) T = refineStep(map, kfs, T).T;
  const auto [rot, trans] = poseError(T, T_true);
  EXPECT_LT(rad2deg(rot), 0.15);
  EXPECT_LT(trans, 0.05);
}

// Reference scans accumulated along a straight drive constrain all six extrinsic DOFs, including
// the roll/pitch/height errors that the yaw/x/y registration stage cannot remove.
TEST(Calibration, RegistrationRecoversSixDof) {
  std::vector<Eigen::Vector3d> scene;
  for (int ix = 0; ix <= 200; ++ix)
    for (int iy = 0; iy <= 106; ++iy) scene.emplace_back(-8.0 + 0.3 * ix, -16.0 + 0.3 * iy, 0.0);
  auto wall = [&](double x0, double y0, double x1, double y1) {
    const int ns = int(std::ceil(std::hypot(x1 - x0, y1 - y0) / 0.2));
    for (int is = 0; is <= ns; ++is) {
      const double s = double(is) / ns;
      for (int iz = 0; iz <= 20; ++iz)
        scene.emplace_back(x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, 1.5 + 0.2 * iz);
    }
  };
  // Separated wall patches keep ground/wall intersections from mixing planes within one voxel.
  wall(-6, 12, 12, 12);
  wall(16, -12, 44, -12);
  wall(18, 4, 18, 10);
  wall(36, 5, 36, 12);
  wall(22, 12, 32, 16);
  wall(30, -9, 38, -5);

  // A nonzero map heading exercises the map/base rotation in the registration Jacobian.
  const double heading = deg2rad(12.0);
  const Eigen::Matrix3d R_map_path = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for (auto& p : scene) p = R_map_path * p;
  const Eigen::Isometry3d T_true = makeIso(3.8, 1.3, 2.1, deg2rad(3.0), deg2rad(-8.0), deg2rad(35.0));
  const Eigen::Isometry3d T_ref = makeIso(1.0, 0.0, 3.2, 0.0, 0.0, deg2rad(7.0));
  std::vector<Keyframe> kfs;
  std::vector<Eigen::Vector3d> reference;
  for (int k = 0; k < 9; ++k) {
    Keyframe kf;
    kf.T_map_base = makeIso(4.0 * k * std::cos(heading), 4.0 * k * std::sin(heading), 0.0, 0.0, 0.0, heading);
    const Eigen::Isometry3d T_map_corner = kf.T_map_base * T_true;
    const Eigen::Isometry3d T_map_ref = kf.T_map_base * T_ref;
    const Eigen::Isometry3d T_corner_map = T_map_corner.inverse(), T_ref_map = T_map_ref.inverse();
    for (const auto& p : scene) {
      const Eigen::Vector3d ps = T_corner_map * p, pr = T_ref_map * p;
      if (pr.norm() < 25.0) reference.push_back(T_map_ref * pr);
      if (ps.norm() < 24.0 && ps.x() > 0.0) kf.pts.push_back(ps);
    }
    kf.pts = voxelDownsample(kf.pts, 0.35);
    ASSERT_GT(kf.pts.size(), 100u);
    kfs.push_back(std::move(kf));
  }
  SurfelMap map;
  map.build(voxelDownsample(reference, 0.2), 1.0, 0.05, 5);
  ASSERT_GT(map.size(), 100u);

  Eigen::Isometry3d T = makeIso(3.88, 1.24, 2.15, deg2rad(3.4), deg2rad(-8.4), deg2rad(36.5));
  for (int it = 0; it < 15; ++it) {
    const StepResult r = refineStep6(map, kfs, T);
    ASSERT_GT(r.matched, 100);
    ASSERT_TRUE(r.T.matrix().allFinite());
    T = r.T;
  }
  const auto [rot, trans] = poseError(T, T_true);
  EXPECT_LT(rad2deg(rot), 0.05);
  EXPECT_LT(trans, 0.01);
}
