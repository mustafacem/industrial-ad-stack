// Small math helpers shared by every package. ROS-free.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace iad {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }
template <class T>
inline T sq(T x) { return x * x; }

// Planar pose of base_link (rear-axle centre on the ground), map frame.
struct Pose2 {
  double x = 0, y = 0, yaw = 0;
};

inline Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// ZYX (yaw-pitch-roll) Euler angles, returned as (roll, pitch, yaw).
inline Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond& q_in) {
  const Eigen::Quaterniond q = q_in.normalized();
  const double sinr = 2 * (q.w() * q.x() + q.y() * q.z()), cosr = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  const double sinp = std::clamp(2 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0);
  const double siny = 2 * (q.w() * q.z() + q.x() * q.y()), cosy = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  return {std::atan2(sinr, cosr), std::asin(sinp), std::atan2(siny, cosy)};
}

inline double yawFromQuat(const Eigen::Quaterniond& q) { return rpyFromQuat(q).z(); }

inline Eigen::Isometry3d makeIso(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = quatFromRPY(roll, pitch, yaw).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// Skew-symmetric matrix [v]x.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

// SE(3) exponential for a twist (omega, t) using the first-order (decoupled) approximation that is
// standard in point-cloud registration: R = exp([omega]x), translation applied directly.
inline Eigen::Isometry3d expSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d w = xi.head<3>();
  const double th = w.norm();
  T.linear() = th < 1e-12 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(th, w / th).toRotationMatrix();
  T.translation() = xi.tail<3>();
  return T;
}

// Rotation error angle [rad] and translation error [m] between two transforms.
inline std::pair<double, double> poseError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Isometry3d d = a.inverse() * b;
  return {Eigen::AngleAxisd(d.linear()).angle(), d.translation().norm()};
}

class Stopwatch {
 public:
  Stopwatch() : t0_(std::chrono::steady_clock::now()) {}
  double ms() const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_).count();
  }
  void reset() { t0_ = std::chrono::steady_clock::now(); }

 private:
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace iad
