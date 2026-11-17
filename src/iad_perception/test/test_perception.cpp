#include <gtest/gtest.h>

#include <random>

#include "iad_perception/dust_filter.hpp"
#include "iad_perception/ground_segmentation.hpp"
#include "iad_perception/objects.hpp"
#include "iad_perception/occupancy.hpp"

using namespace iad::perception;

// Builds an organized scan (rings x 1200 columns) of a dark (mud-like) wall at 20 m; in a sector, half of
// the beams return early from a dust cloud at a random 5..15 m instead.
static Scan dustyWall(uint32_t seed, std::vector<uint8_t>& is_dust, double dust_intensity) {
  Scan s;
  s.rings = 16, s.cols = 1200;
  s.image.assign(size_t(s.rings) * size_t(s.cols), -1);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(0, 1);
  is_dust.clear();
  for (int ring = 0; ring < s.rings; ++ring)
    for (int a = 0; a < s.cols; ++a) {
      const double az = (a + 0.5) / s.cols * 2 * kPi - kPi, el = (-15.0 + 2.0 * ring) * kPi / 180;
      const bool dusty = az > 0 && az < 0.6 && U(rng) < 0.5;
      const double r = dusty ? 5 + 10 * U(rng) : 20.0;
      LidarPoint p;
      p.x = float(r * std::cos(el) * std::cos(az)), p.y = float(r * std::cos(el) * std::sin(az)), p.z = float(r * std::sin(el));
      p.intensity = dusty ? float(dust_intensity) : 8.f;  // the wall is as dark as dust
      p.ring = uint16_t(ring), p.col = uint16_t(a);
      s.image[size_t(ring) * size_t(s.cols) + size_t(a)] = int(s.pts.size());
      s.pts.push_back(p), is_dust.push_back(dusty);
    }
  return s;
}

static std::vector<Eigen::Vector3f> asMap(const Scan& s) {
  std::vector<Eigen::Vector3f> m;
  for (const auto& p : s.pts) m.emplace_back(p.x, p.y, p.z);
  return m;
}

// Intensity alone would delete the dark wall; range-image consistency keeps it and removes the dust,
// including bright dust echoes (strongly isolated) and across sweeps (temporal memory on).
TEST(Dust, RemovesDustKeepsDarkSurface) {
  DustFilter f;
  for (uint32_t sweep = 0; sweep < 4; ++sweep) {
    std::vector<uint8_t> truth;
    const Scan s = dustyWall(sweep + 1, truth, sweep % 2 ? 40.0 : 4.0);
    const auto d = f.classify(s, asMap(s));
    size_t dust = 0, dust_hit = 0, wall = 0, wall_lost = 0;
    for (size_t i = 0; i < s.pts.size(); ++i) {
      if (truth[i]) ++dust, dust_hit += d[i];
      else ++wall, wall_lost += d[i];
    }
    EXPECT_GT(double(dust_hit) / dust, 0.85) << "sweep " << sweep;
    EXPECT_LT(double(wall_lost) / wall, 0.01) << "sweep " << sweep;
  }
}

// An organized PointCloud2 (Gazebo gpu_lidar layout) parses into a range image; inf = no echo.
TEST(Types, OrganizedCloudToRangeImage) {
  sensor_msgs::msg::PointCloud2 m;
  m.height = 2, m.width = 3, m.point_step = 16, m.row_step = 48;
  for (auto [n, off] : {std::pair<const char*, int>{"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}}) {
    sensor_msgs::msg::PointField f;
    f.name = n, f.offset = uint32_t(off), f.datatype = sensor_msgs::msg::PointField::FLOAT32, f.count = 1;
    m.fields.push_back(f);
  }
  m.data.resize(96);
  for (int k = 0; k < 6; ++k) {
    float v[4] = {float(k + 1), 0.f, 0.f, 50.f};
    if (k == 4) v[0] = std::numeric_limits<float>::infinity();
    std::memcpy(&m.data[size_t(k) * 16], v, 16);
  }
  const Scan s = scanFromMsg(m);
  EXPECT_EQ(s.rings, 2);
  EXPECT_EQ(s.cols, 3);
  EXPECT_EQ(s.pts.size(), 5u);
  EXPECT_EQ(s.at(1, 1), -1);  // the inf return
  EXPECT_FLOAT_EQ(s.pts[size_t(s.at(1, 2))].x, 6.f);
}

// Hill + ramp, gravity-aligned. CZM keeps the slopes as ground and finds the boulder; one RANSAC plane can't.
TEST(Ground, CzmHandlesSlopesRansacDoesNot) {
  std::vector<Eigen::Vector3f> P;
  std::vector<uint8_t> is_obst;
  for (double x = -60; x <= 60; x += 0.4)
    for (double y = -60; y <= 60; y += 0.4) {
      const double r = std::hypot(x, y);
      if (r < 3 || r > 75) continue;
      const double z = 0.12 * std::max(0.0, x - 10) + 2.0 * std::sin(y / 25.0);  // ramp + rolling terrain
      P.emplace_back(float(x), float(y), float(z));
      is_obst.push_back(0);
      if (std::hypot(x - 20, y - 5) < 1.2) P.emplace_back(float(x), float(y), float(z + 1.2)), is_obst.push_back(1);
    }
  const auto czm = segmentGroundCzm(P, {}, CzmParams{});
  const auto rs = segmentGroundRansac(P, {}, 0.25, 200, 3);
  size_t g = 0, g_czm = 0, g_rs = 0, o = 0, o_czm = 0;
  for (size_t i = 0; i < P.size(); ++i) {
    if (is_obst[i]) ++o, o_czm += !czm.ground[i];
    else ++g, g_czm += czm.ground[i], g_rs += rs.ground[i];
  }
  EXPECT_GT(double(g_czm) / g, 0.95);
  EXPECT_LT(double(g_rs) / g, 0.7);
  EXPECT_GT(double(o_czm) / o, 0.9);
}

TEST(Objects, MinAreaRectRecoversRotatedBox) {
  std::vector<Eigen::Vector2d> pts;
  const double yaw = 0.4;
  for (double u = -3; u <= 3; u += 0.1)
    for (double v = -1; v <= 1; v += 0.1)
      pts.emplace_back(10 + std::cos(yaw) * u - std::sin(yaw) * v, 5 + std::sin(yaw) * u + std::cos(yaw) * v);
  const Obb b = minAreaRect(pts);
  EXPECT_NEAR(b.length, 6.0, 0.15);
  EXPECT_NEAR(b.width, 2.0, 0.15);
  EXPECT_NEAR(std::abs(std::cos(b.yaw - yaw)), 1.0, 1e-3);
  EXPECT_NEAR(b.c.x(), 10, 0.05);
}

TEST(Occupancy, LeakedDustInsideAPlumeNeverBlocksButAnObjectInItDoes) {
  LogOddsGrid g(LogOddsGrid::Params{});
  g.recenter(0, 0);
  const Eigen::Vector2d o(0, 0), leak(15, 0), rock(15, 6);
  std::vector<Eigen::Vector2d> dust;
  for (int k = 0; k < 12; ++k) dust.push_back({15.0 + 0.1 * (k % 4), 0.1 * (k / 4)});  // dense plume at the leak
  for (int k = 0; k < 3; ++k) dust.push_back({15.0, 6.2});                          // a little dust at the rock
  for (int k = 0; k < 40; ++k) g.integrate(o, {leak, rock, rock + Eigen::Vector2d(0.1, 0), rock + Eigen::Vector2d(0, 0.1)}, {}, dust);
  int i, j;
  ASSERT_TRUE(g.toCell(leak, i, j));
  EXPECT_FALSE(g.occupied(g.idx(i, j)));  // re-hit 40 sweeps in a row, but dust-dominated
  ASSERT_TRUE(g.toCell(rock, i, j));
  EXPECT_TRUE(g.occupied(g.idx(i, j)));
}

TEST(Occupancy, TransientHitDecaysPersistentHitStays) {
  LogOddsGrid g(LogOddsGrid::Params{});
  g.recenter(0, 0);
  const Eigen::Vector2d o(0, 0), wall(20, 0), puff(10, 3), behind(30, 3);
  g.integrate(o, {wall, puff}, {});
  for (int k = 0; k < 6; ++k) g.integrate(o, {wall}, {behind});
  int i, j;
  ASSERT_TRUE(g.toCell(wall, i, j));
  EXPECT_TRUE(g.occupied(g.idx(i, j)));
  ASSERT_TRUE(g.toCell(puff, i, j));
  EXPECT_FALSE(g.occupied(g.idx(i, j)));
}
