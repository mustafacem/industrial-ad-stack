// Offline perception evaluation: drive the haul route in the quarry, scan, and score the baseline and
// this stack against per-point ground truth, broken down by range. Deterministic.
//   perception_eval [--step 4] [--seed 7] [--dump N]   (dump: write frame N as CSV for plotting)
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include "iad_common/vehicle.hpp"
#include "iad_perception/dust_filter.hpp"
#include "iad_perception/ground_segmentation.hpp"
#include "iad_sim/sensors.hpp"
#include "iad_sim/vehicle_sim.hpp"
#include "iad_sim/viz.hpp"

using namespace iad;
using namespace iad::sim;
using namespace iad::perception;

int main(int argc, char** argv) {
  double step = 4.0;
  int seed = 7, dump = -1;
  for (int i = 1; i + 1 < argc; ++i) {
    if (!std::strcmp(argv[i], "--step")) step = std::atof(argv[i + 1]);
    if (!std::strcmp(argv[i], "--seed")) seed = std::atoi(argv[i + 1]);
    if (!std::strcmp(argv[i], "--dump")) dump = std::atoi(argv[i + 1]);
  }
  World w = makeQuarry(uint32_t(seed));
  std::mt19937 mt(static_cast<uint32_t>(seed));
  const Route* route = w.route("haul");
  const VehicleParams vp = haulTruckParams();
  LidarConfig cfg;
  cfg.T_base_true = makeIso(5.2, 0, 4.15, 0, 0, 0);
  const auto ego = egoBodyBoxes(vp);
  // conf[pipeline][range bucket][label][category]
  const double buckets[] = {0, 15, 30, 50, 80};
  static uint64_t conf[2][4][6][4];
  std::memset(conf, 0, sizeof(conf));
  int frame = 0;
  double acc = 0;
  for (size_t k = 1; k < route->pts.size(); ++k) {
    acc += (route->pts[k] - route->pts[k - 1]).norm();
    for (int s = 0; s < 40; ++s) w.step(0.01, mt);  // let actors and dust evolve between frames
    if (acc < step) continue;
    acc = 0;
    const Eigen::Vector2d p = route->pts[k], q = route->pts[std::min(k + 3, route->pts.size() - 1)];
    const double yaw = std::atan2(q.y() - p.y(), q.x() - p.x());
    VehicleSim veh(vp, 0.01);
    veh.reset({p.x(), p.y(), yaw}, w.terrain);
    const Eigen::Isometry3d T_mb = veh.T_map_base();
    std::vector<Box> dyn = w.actorBoxes();
    for (Box b : ego) {
      const Eigen::Vector3d l = b.c;
      b.c = Eigen::Vector3d(p.x() + std::cos(yaw) * l.x() - std::sin(yaw) * l.y(), p.y() + std::sin(yaw) * l.x() + std::cos(yaw) * l.y(), veh.z + l.z());
      b.yaw = yaw;
      dyn.push_back(b);
    }
    const Cloud c = scanLidar(w, dyn, T_mb * cfg.T_base_true, cfg, uint64_t(seed) * 1000 + uint64_t(frame));
    const size_t n = c.size();
    const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(T_mb.linear()));
    const Eigen::Matrix3d R_lb = (Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    std::vector<Eigen::Vector3f> P(n), S(n);
    std::vector<uint8_t> self(n);
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d pl(c[i].x, c[i].y, c[i].z), pb = cfg.T_base_true * pl;
      self[i] = (pb.x() > -vp.rear - 0.6 && pb.x() < vp.front + 0.6 && std::abs(pb.y()) < vp.width / 2 + 0.6 && pb.z() < vp.height + 1.2) || pl.norm() > 80;
      P[i] = (R_lb * pb).cast<float>(), S[i] = pl.cast<float>();
    }
    const auto dust = classifyDust(c, DustParams{});
    std::vector<uint8_t> ign(n);
    for (size_t i = 0; i < n; ++i) ign[i] = self[i] || dust[i];
    const auto czm = segmentGroundCzm(P, ign, CzmParams{}, R_lb * Eigen::Vector3d::UnitZ());
    const auto rs = segmentGroundRansac(S, self, 0.25, 150, 42);
    std::ofstream out;
    if (frame == dump) out.open("/ws/output/logs/perception_frame.csv"), out << "x,y,z,label,cat_ours,cat_naive,h\n";
    for (size_t i = 0; i < n; ++i) {
      if (self[i] || c[i].label == uint8_t(Label::kEgo)) continue;
      const double r = std::hypot(P[i].x(), P[i].y());
      int b = 0;
      while (b < 3 && r >= buckets[b + 1]) ++b;
      const int L = std::min<int>(5, c[i].label);
      const int cat_o = dust[i] ? 2 : czm.ground[i] ? 0 : (czm.height[i] > vp.height + 1.0 ? 3 : 1);
      const int cat_n = rs.ground[i] ? 0 : 1;
      conf[0][b][L][cat_n]++, conf[1][b][L][cat_o]++;
      if (out.is_open()) out << P[i].x() << "," << P[i].y() << "," << P[i].z() << "," << L << "," << cat_o << "," << cat_n << "," << czm.height[i] << "\n";
    }
    ++frame;
  }
  const char* lab[] = {"ground", "static", "dynamic", "dust", "ego", "steep"};
  for (int pl = 0; pl < 2; ++pl) {
    std::printf("\n== %s (%d frames)  rows: truth, cols: ground/obstacle/dust/overhang (%%)\n", pl ? "THIS STACK" : "BASELINE", frame);
    for (int b = 0; b < 4; ++b) {
      std::printf(" range %2.0f-%2.0f m\n", buckets[b], buckets[b + 1]);
      for (int L : {0, 5, 1, 2, 3}) {
        uint64_t t = 0;
        for (int k = 0; k < 4; ++k) t += conf[pl][b][L][k];
        if (!t) continue;
        std::printf("   %-8s n=%8lu  %6.1f %6.1f %6.1f %6.1f\n", lab[L], (unsigned long)t, 100.0 * conf[pl][b][L][0] / t,
                    100.0 * conf[pl][b][L][1] / t, 100.0 * conf[pl][b][L][2] / t, 100.0 * conf[pl][b][L][3] / t);
      }
    }
  }
  return 0;
}
