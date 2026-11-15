#include <gtest/gtest.h>

#include "iad_gz_sim/geodesy.hpp"
#include "iad_gz_sim/gnss_model.hpp"
#include "iad_gz_sim/terrain_grid.hpp"
#include "iad_gz_sim/world.hpp"

using namespace iad::sim;

TEST(Geodesy, RoundTrip) {
  iad::geo::Origin o(35.6206, 139.79, 0.0);
  const Eigen::Vector3d p(312.4, -87.1, 12.3);
  double lat, lon, alt;
  o.toGeodetic(p, lat, lon, alt);
  EXPECT_LT((o.toEnu(lat, lon, alt) - p).norm(), 1e-3);
}

TEST(World, BothSitesBuild) {
  for (const char* n : {"quarry", "port"}) {
    World w = makeWorld(n, 7);
    EXPECT_GT(w.terrain.nx, 100);
    EXPECT_FALSE(w.poses.empty());
    EXPECT_FALSE(w.routes.empty());
  }
}

TEST(Gnss, OpenSkyFixVsUnderCrane) {
  World w = makePort(7);
  // Far from any structure the sky is clear; under the quay crane boom it is not.
  const double open = GnssModel::skyBlocked(w, Eigen::Vector3d(-40, -60, 4));
  const auto* crane = w.pose("crane2_lane");
  ASSERT_NE(crane, nullptr);
  const double under = GnssModel::skyBlocked(w, Eigen::Vector3d(crane->x, crane->y, 5));
  EXPECT_LT(open, 0.1);
  EXPECT_GT(under, open + 0.1);
}
