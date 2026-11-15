// Sensor models: spinning multi-beam LiDAR (dual return, dust backscatter, labels), GNSS whose
// quality follows sky visibility in the 3D world, IMU with bias, wheel odometry.
#pragma once
#include <string>
#include <vector>

#include "iad_common/math.hpp"
#include "iad_common/point_types.hpp"
#include "iad_sim/world.hpp"

namespace iad::sim {

struct LidarConfig {
  std::string name = "top", frame = "lidar_top";
  Eigen::Isometry3d T_base_true = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_base_nominal = Eigen::Isometry3d::Identity();  // what the TF tree claims
  int channels = 32;
  double vmin = -25, vmax = 15, hres = 0.3;  // [deg]
  double range = 120, min_range = 1.2, noise = 0.02;
  bool dual = true;
};

struct ScanStats {
  double ms = 0;
  size_t points = 0, dust = 0;
};

Cloud scanLidar(const World& w, const std::vector<Box>& dynamic, const Eigen::Isometry3d& T_map_lidar,
                const LidarConfig& cfg, uint64_t seed, ScanStats* stats = nullptr);

enum class GnssStatus : int { kNoFix = -1, kSingle = 0, kFloat = 1, kFix = 2 };
const char* gnssStatusName(GnssStatus s);

struct GnssSample {
  bool valid = false;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double sigma_reported = 1.0;
  GnssStatus status = GnssStatus::kNoFix;
  double sky_blocked = 0;
  double error = 0;  // true horizontal error [m] (for evaluation)
};

class GnssModel {
 public:
  GnssSample sample(const World& w, const std::vector<Box>& dynamic, const Eigen::Vector3d& antenna, double dt,
                    FastRng& rng);
  static double skyBlocked(const World& w, const std::vector<Box>& dynamic, const Eigen::Vector3d& antenna);

 private:
  Eigen::Vector2d bias_ = Eigen::Vector2d::Zero();
  GnssStatus status_ = GnssStatus::kFix;
  double upgrade_timer_ = 0;
};

struct ImuModel {
  Eigen::Vector3d gyro_bias{0.0015, -0.001, 0.0035};  // [rad/s] (0.2 deg/s on z: dead reckoning drifts)
  Eigen::Vector3d accel_bias{0.02, -0.03, 0.01};
  double gyro_noise = 0.002, accel_noise = 0.04;
  void sample(const Eigen::Vector3d& w_true, const Eigen::Vector3d& f_true, FastRng& rng, Eigen::Vector3d& w,
              Eigen::Vector3d& f) const {
    w = w_true + gyro_bias + gyro_noise * Eigen::Vector3d(rng.normal(), rng.normal(), rng.normal());
    f = f_true + accel_bias + accel_noise * Eigen::Vector3d(rng.normal(), rng.normal(), rng.normal());
  }
};

}  // namespace iad::sim
