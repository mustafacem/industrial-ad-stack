// Control-local helpers (this package shares no code with the other subsystems).
#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace iad::control {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;
inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double rad2deg(double r) { return r * 180.0 / kPi; }
inline Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return {std::atan2(R(2, 1), R(2, 2)), std::asin(std::clamp(-R(2, 0), -1.0, 1.0)), std::atan2(R(1, 0), R(0, 0))};
}

struct Stopwatch {
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  double ms() const { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); }
};

// What the controller knows about the vehicle (its own parameter file; base_link = rear axle).
struct VehicleParams {
  double wheelbase = 4.3;
  double max_steer = 0.6, max_steer_rate = 0.35, steer_tau = 0.25;  // [rad], [rad/s], [s]
  double delay = 0.4, accel_tau = 0.6;                              // actuator dead time / drivetrain lag [s]
  double max_acc = 2.0, max_dec = 2.5;                              // [m/s^2]
};

}  // namespace iad::control
