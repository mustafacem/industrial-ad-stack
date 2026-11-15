#include <gtest/gtest.h>

#include "iad_sim/sensors.hpp"
#include "iad_sim/vehicle_sim.hpp"

using namespace iad;
using namespace iad::sim;

TEST(RayBox, HitsFrontFaceWithOutwardNormal) {
  Box b;
  b.c = {10, 0, 1}, b.half = {1, 1, 1};
  double t;
  Eigen::Vector3d n;
  ASSERT_TRUE(rayBox(b, {0, 0, 1}, {1, 0, 0}, t, n));
  EXPECT_NEAR(t, 9.0, 1e-9);
  EXPECT_NEAR(n.x(), -1.0, 1e-9);
}

TEST(Terrain, RaycastFlatGround) {
  Terrain t;
  t.init(201, 201, 0.5, -50, -50);
  t.updateBounds();
  double th;
  const Eigen::Vector3d d = Eigen::Vector3d(1, 0, -0.1).normalized();
  ASSERT_TRUE(t.raycast({0, 0, 2}, d, 100, th));
  EXPECT_NEAR((Eigen::Vector3d(0, 0, 2) + th * d).z(), 0.0, 1e-3);
}

TEST(Lidar, QuarryScanHasGroundAndLabels) {
  World w = makeQuarry(7);
  const auto* s = w.pose("start");
  ASSERT_NE(s, nullptr);
  LidarConfig cfg;
  const Eigen::Isometry3d T = makeIso(s->x, s->y, w.terrain.height(s->x, s->y) + 4.2, 0, 0, s->yaw);
  ScanStats st;
  const Cloud c = scanLidar(w, {}, T, cfg, 1, &st);
  size_t ground = 0;
  for (const auto& p : c) ground += p.label == uint8_t(Label::kGround);
  EXPECT_GT(c.size(), 15000u);
  EXPECT_GT(ground, c.size() / 2);
}

TEST(Vehicle, DeadTimeDelaysResponse) {
  World w = makeQuarry(7);
  VehicleParams p = haulTruckParams();
  VehicleSim v(p, 0.01);
  const auto* s = w.pose("start");
  v.reset({s->x, s->y, s->yaw}, w.terrain);
  FastRng rng(3);
  for (int k = 0; k < 30; ++k) v.step({0.3, 0.0}, w, rng);  // 0.3 s < 0.4 s dead time
  EXPECT_NEAR(v.steer, 0.0, 1e-9);
  for (int k = 0; k < 200; ++k) v.step({0.3, 0.0}, w, rng);
  EXPECT_NEAR(v.steer, 0.3, 0.02);
}
