// GNSS degradation from sky visibility in the 3D site (cranes, container stacks, pit walls):
// RTK fix in open sky, float with multipath bias near structure, single / no fix under it.
#pragma once
#include "iad_gz_sim/rng.hpp"
#include "iad_gz_sim/world.hpp"

namespace iad::sim {

enum class GnssStatus : int { kNoFix = -1, kSingle = 0, kFloat = 1, kFix = 2 };
const char* gnssStatusName(GnssStatus s);

struct GnssSample {
  bool valid = false;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double sigma_reported = 1.0;
  GnssStatus status = GnssStatus::kNoFix;
  double sky_blocked = 0;
  double error = 0;  // true horizontal error [m] (evaluation only)
};

class GnssModel {
 public:
  GnssSample sample(const World& w, const Eigen::Vector3d& antenna, double dt, FastRng& rng);
  static double skyBlocked(const World& w, const Eigen::Vector3d& antenna);

 private:
  Eigen::Vector2d bias_ = Eigen::Vector2d::Zero();
  GnssStatus status_ = GnssStatus::kFix;
  double upgrade_timer_ = 0;
};

}  // namespace iad::sim
