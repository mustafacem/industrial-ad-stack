// Offline planning evaluation on the real site costmaps: plan time, expansions, length, cusps, and the
// curvature profile of the result (a zig-zag path shows up as curvature sign flips).
//   planning_eval <port|quarry> x0 y0 yaw0 x1 y1 yaw1 [--csv out.csv]
#include <cstdio>
#include <cstring>
#include <fstream>

#include "iad_planning/hybrid_astar.hpp"
#include "iad_planning/velocity.hpp"
#include "iad_sim/maps.hpp"

using namespace iad;
using namespace iad::planning;

int main(int argc, char** argv) {
  if (argc < 8) return std::printf("usage: planning_eval <port|quarry> x0 y0 yaw0 x1 y1 yaw1 [--csv f]\n"), 1;
  const std::string scen = argv[1];
  sim::World w = sim::makeWorld(scen, 7);
  const CostGrid g = sim::buildCostmap(w, 0.5, 5.5);
  const VehicleParams v = scen == "port" ? containerAgvParams() : haulTruckParams();
  HybridAStar planner(v, PlannerParams{});
  PlanStats st;
  const Pose2 a{std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4])}, b{std::atof(argv[5]), std::atof(argv[6]), std::atof(argv[7])};
  const Path p = planner.plan(g, a, 0, b, &st);
  std::printf("status %s  expansions %d  %.0f ms  rs_shot %d (from index %zu of %zu)\n", st.status.c_str(), st.expansions, st.ms,
              st.rs_shot, st.rs_begin, p.size());
  if (p.empty()) return 2;
  const Trajectory t = timeParameterize(p, g, SpeedParams{});
  int flips = 0, cusps = 0;
  double kmax = 0;
  for (size_t i = 1; i < t.size(); ++i) {
    cusps += p[i].dir != p[i - 1].dir;
    kmax = std::max(kmax, std::abs(t[i].kappa));
    if (i > 1 && t[i].kappa * t[i - 1].kappa < 0 && std::abs(t[i].kappa) > 0.01 && std::abs(t[i - 1].kappa) > 0.01) ++flips;
  }
  std::printf("length %.1f m  cusps %d  max curvature %.3f 1/m (vehicle min radius %.1f m)  curvature sign flips %d\n", t.back().s,
              cusps, kmax, v.wheelbase / std::tan(v.max_steer), flips);
  for (int i = 8; i + 1 < argc; ++i)
    if (!std::strcmp(argv[i], "--csv")) {
      std::ofstream f(argv[i + 1]);
      f << "s,x,y,yaw,kappa,v,dir\n";
      for (const auto& q : t) f << q.s << "," << q.x << "," << q.y << "," << q.yaw << "," << q.kappa << "," << q.v << "," << q.dir << "\n";
    }
  return 0;
}
