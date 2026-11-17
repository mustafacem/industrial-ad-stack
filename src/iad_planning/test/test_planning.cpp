#include <gtest/gtest.h>

#include <random>

#include "iad_planning/behaviour.hpp"
#include "iad_planning/hybrid_astar.hpp"
#include "iad_planning/reeds_shepp.hpp"
#include "iad_planning/velocity.hpp"

using namespace iad::planning;

// Every sampled Reeds-Shepp path must end exactly on the goal pose, and its length must be at least the
// straight-line distance: this exercises all 48 path words and their symmetries.
TEST(ReedsShepp, PathsEndOnTheGoal) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> U(-30, 30), A(-M_PI, M_PI);
  const ReedsShepp rs(7.0);
  int forward_found = 0;
  for (int k = 0; k < 2000; ++k) {
    const double x0 = U(rng), y0 = U(rng), t0 = A(rng), x1 = U(rng), y1 = U(rng), t1 = A(rng);
    double len = 0;
    const auto s = rs.sample(x0, y0, t0, x1, y1, t1, 0.25, &len);
    ASSERT_FALSE(s.empty());
    EXPECT_NEAR(s.back().x, x1, 1e-3);
    EXPECT_NEAR(s.back().y, y1, 1e-3);
    EXPECT_NEAR(std::abs(wrapAngle(s.back().yaw - t1)), 0.0, 1e-3);
    EXPECT_GE(len + 1e-6, std::hypot(x1 - x0, y1 - y0));
    // Forward-only word: when one exists it is exact, never shorter, never reverses. (Reeds-Shepp words
    // turn at most pi per arc, so some pairs only have a forward path that loops: then it is empty.)
    double lf = 0;
    const auto f = rs.sample(x0, y0, t0, x1, y1, t1, 0.25, &lf, +1);
    forward_found += !f.empty();
    if (f.empty()) continue;
    EXPECT_NEAR(f.back().x, x1, 1e-3);
    EXPECT_NEAR(f.back().y, y1, 1e-3);
    EXPECT_GE(lf + 1e-6, len);
    for (const auto& q : f) EXPECT_EQ(q.dir, 1);
  }
  EXPECT_GT(forward_found, 1000);  // ~56 % of uniformly random pose pairs (the rest need a loop)
}

// A dead-end bay: the truck must back into it. Path must be collision-free, end on the goal, use reverse.
TEST(HybridAStar, BacksIntoBay) {
  CostGrid g(240, 160, 0.5, 0, 0, 0);
  for (int j = 0; j < g.h; ++j)
    for (int i = 0; i < g.w; ++i) {
      const double x = g.cx(i), y = g.cy(j);
      const bool bay_wall = x > 90 && (std::abs(y - 40) > 4.5 && std::abs(y - 40) < 6.0);
      const bool bay_end = x > 113;
      if (bay_wall || bay_end || y < 10 || y > 70) g.at(i, j) = kLethal;
    }
  const VehicleParams v = VehicleParams{};
  HybridAStar planner(v, PlannerParams{});
  PlanStats st;
  const Path p = planner.plan(g, {30, 40, 0.0}, 1, {108, 40, M_PI}, &st);
  ASSERT_FALSE(p.empty()) << st.status;
  EXPECT_NEAR(p.back().x, 108, 0.5);
  EXPECT_NEAR(p.back().y, 40, 0.5);
  bool reverse = false;
  for (const auto& q : p) {
    reverse = reverse || q.dir < 0;
    EXPECT_FALSE(footprintCollides(g, q.x, q.y, q.yaw, v, 0.0));
  }
  EXPECT_TRUE(reverse);
  const Trajectory t = timeParameterize(p, g, SpeedParams{});
  for (size_t i = 1; i + 1 < p.size(); ++i)
    if (p[i + 1].dir != p[i].dir) EXPECT_NEAR(t[i].v, 0.0, 1e-9);  // stop at every cusp
  EXPECT_NEAR(t.back().v, 0.0, 1e-9);
}

// Behaviour layer on a straight 100 m lane at 7 m/s (ego 4.2 m wide, front 6.6 m, rear 2.8 m).
static Trajectory straightLane() {
  Trajectory t;
  for (int i = 0; i <= 100; ++i) {
    TrajPoint p;
    p.x = i, p.s = i, p.v = 7.0, p.dir = 1;
    t.push_back(p);
  }
  return t;
}

TEST(Behaviour, FollowsPassesYieldsAndSlowsInDust) {
  const Trajectory t = straightLane();
  const BehaviourParams bp;
  auto decide = [&](std::vector<Agent> a, int dust = 0) { return decideBehaviour(t, 2.1, 6.6, 2.8, 7.0, a, dust, bp); };
  EXPECT_EQ(decide({}).name, "cruise");
  // A slower truck 30 m ahead in the lane: follow it, never closer than follow_min.
  const Behaviour f = decide({{30, 0.3, 4.0, 0, 9, 4}});
  EXPECT_EQ(f.name, "follow");
  EXPECT_LT(f.v_eff, 7.0);
  EXPECT_NEAR(f.s_stop, 30 - 4.5 - bp.follow_min - 6.6, 1e-6);
  // An oncoming truck in the other lane (9 m to the left): pass it at pass_v.
  const Behaviour p = decide({{35, 9.0, -6.0, 0, 9, 4}});
  EXPECT_EQ(p.name, "passing");
  EXPECT_DOUBLE_EQ(p.v_eff, bp.pass_v);
  // A pickup 12 m to the right, crossing towards the lane: stop before its crossing line.
  const Behaviour y = decide({{25, -12.0, 0, 6.0, 5, 2}});
  EXPECT_EQ(y.name, "yield");
  EXPECT_LT(y.s_stop, 25 - 1 - 6.6);
  EXPECT_EQ(decide({{25, -12.0, 0, -6.0, 5, 2}}).name, "cruise");  // driving away from the lane
  // Dust in the lane ahead.
  EXPECT_EQ(decide({}, 700).name, "dust");
  EXPECT_DOUBLE_EQ(decide({}, 700).v_eff, bp.dust_v_heavy);
  // The capped profile stops at the yield line and never asks for more than the cap.
  const Trajectory c = applyBehaviour(t, y, 7.0, bp);
  for (const auto& q : c) {
    if (q.s >= y.s_stop) {
      EXPECT_DOUBLE_EQ(q.v, 0.0);
    }
    EXPECT_LE(q.v, 7.0 + 1e-9);
  }
}
