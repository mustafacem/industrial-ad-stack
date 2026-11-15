// Heavy-vehicle dynamics on a height field. Kinematic bicycle in the plane plus what makes off-road
// heavy vehicles hard: actuator dead time, rate-limited lagged steering, power-limited traction,
// grade + rolling resistance, slip on loose ground, and pitch/roll from the terrain under the wheels.
#pragma once
#include <deque>

#include "iad_common/vehicle.hpp"
#include "iad_sim/world.hpp"

namespace iad::sim {

struct ActuatorCmd {
  double steer = 0;  // [rad] road-wheel angle
  double accel = 0;  // [m/s^2] longitudinal, along body x
  int gear = 1;      // +1 drive, -1 reverse, 0 neutral: traction only in the selected gear, else brake
};

class VehicleSim {
 public:
  VehicleSim(const VehicleParams& p, double dt) : p_(p), dt_(dt) {}

  void reset(const Pose2& start, const Terrain& t) {
    pose = start, v = v_ground = steer = acc_lag_ = yaw_rate = slip = 0;
    pipe_.assign(size_t(std::lround(p_.delay / dt_)), ActuatorCmd{});
    updateAttitude(t, true);
    q_prev_ = q, vel_prev_ = Eigen::Vector3d::Zero();
  }
  void setLoaded(bool l) { loaded_ = l; }
  bool loaded() const { return loaded_; }
  double mass() const { return p_.mass_empty + (loaded_ ? p_.payload : 0.0); }
  const VehicleParams& params() const { return p_; }

  void step(const ActuatorCmd& cmd, const World& w, FastRng& rng) {
    pipe_.push_back(cmd);
    const ActuatorCmd c = pipe_.front();
    pipe_.pop_front();
    // Steering: first-order lag, rate-limited (hydraulics).
    const double target = std::clamp(c.steer, -p_.max_steer, p_.max_steer);
    const double ds = std::clamp((target - steer) * std::min(1.0, dt_ / p_.steer_tau), -p_.max_steer_rate * dt_,
                                 p_.max_steer_rate * dt_);
    steer += ds;
    // Drivetrain / brake lag, then force limits.
    acc_lag_ += (c.accel - acc_lag_) * std::min(1.0, dt_ / p_.accel_tau);
    // Traction only in the selected gear and never against the current motion; anything else is braking.
    const double m = mass(), dir = std::abs(v) > 0.05 ? (v > 0 ? 1.0 : -1.0) : (c.gear != 0 ? double(c.gear) : 1.0);
    const bool traction = c.gear != 0 && double(c.gear) * dir > 0 && acc_lag_ * dir > 0;
    double a_cmd;
    if (traction) {
      const double a_pow = p_.power / (m * std::max(std::abs(v), 1.5));
      a_cmd = dir * std::min({std::abs(acc_lag_), p_.max_acc * std::sqrt(p_.mass_empty / m), a_pow});
    } else {
      a_cmd = -dir * std::min(std::abs(acc_lag_), p_.max_dec);  // brake: always opposes the motion
    }
    const auto& mi = materialInfo(w.terrain.material(pose.x, pose.y));
    const double a_grav = kGravity * std::sin(pitch);  // pitch < 0 (nose up) slows forward motion
    const double resist = mi.rr * kGravity * std::cos(pitch) * dt_;
    double vn = v + (a_cmd + a_grav) * dt_;
    vn = std::abs(vn) <= resist ? 0.0 : vn - resist * (vn > 0 ? 1.0 : -1.0);
    const bool standstill = std::abs(v) < 0.02;
    if (!standstill && vn * v < 0 && !traction) vn = 0;                 // brakes stop, they don't reverse
    if (standstill && !traction) vn = 0;                                // parked on the brakes, holds on grades
    if (standstill && traction && (a_cmd + a_grav) * dir <= 0) vn = 0;  // too little traction: hill hold
    v = vn;
    // Slip: wheels turn faster than the ground moves on loose material; mud also understeers.
    slip = std::clamp(mi.slip * std::min(1.0, std::abs(v) / 2.0) * (1.0 + 0.25 * rng.normal()), 0.0, 0.5);
    v_ground = v * (1.0 - slip);
    const double k_us = 0.0012 + (mi.slip > 0.1 ? 0.006 : 0.0);
    yaw_rate = v_ground * std::tan(steer) / p_.wheelbase / (1.0 + k_us * v_ground * v_ground);
    pose.x += v_ground * std::cos(pose.yaw) * dt_;
    pose.y += v_ground * std::sin(pose.yaw) * dt_;
    pose.yaw = wrapAngle(pose.yaw + yaw_rate * dt_);
    accel_applied = a_cmd;
    updateAttitude(w.terrain, false);
  }

  Eigen::Isometry3d T_map_base() const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(pose.x, pose.y, z);
    return T;
  }

  // True state (read by sensors and ground-truth publishers).
  Pose2 pose;
  double z = 0, roll = 0, pitch = 0;
  double v = 0;         // wheel speed (what odometry measures)
  double v_ground = 0;  // speed over ground
  double steer = 0, accel_applied = 0, yaw_rate = 0, slip = 0;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d omega_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d specific_force = Eigen::Vector3d(0, 0, kGravity);

 private:
  void updateAttitude(const Terrain& t, bool init) {
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw), L = p_.wheelbase, T = p_.track / 2;
    auto hAt = [&](double a, double b) { return t.height(pose.x + a * c - b * s, pose.y + a * s + b * c); };
    const double hfl = hAt(L, T), hfr = hAt(L, -T), hrl = hAt(0, T), hrr = hAt(0, -T);
    const double pitch_t = std::atan2(0.5 * (hrl + hrr) - 0.5 * (hfl + hfr), L);
    const double roll_t = std::atan2(0.5 * (hfl + hrl) - 0.5 * (hfr + hrr), 2 * T);
    const double z_t = 0.5 * (hrl + hrr);
    const double a = init ? 1.0 : std::min(1.0, dt_ / 0.12);  // suspension low-pass
    pitch += a * (pitch_t - pitch), roll += a * (roll_t - roll), z += a * (z_t - z);
    q = quatFromRPY(roll, pitch, pose.yaw);
    if (init) return;
    const Eigen::Vector3d vel = q * Eigen::Vector3d(v_ground, 0, 0);
    specific_force = q.inverse() * ((vel - vel_prev_) / dt_ + Eigen::Vector3d(0, 0, kGravity));
    const Eigen::AngleAxisd d(q_prev_.inverse() * q);
    omega_body = d.axis() * (d.angle() / dt_);
    if (!std::isfinite(omega_body.x())) omega_body.setZero();
    q_prev_ = q, vel_prev_ = vel;
  }

  VehicleParams p_;
  double dt_;
  std::deque<ActuatorCmd> pipe_;
  double acc_lag_ = 0;
  bool loaded_ = false;
  Eigen::Quaterniond q_prev_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d vel_prev_ = Eigen::Vector3d::Zero();
};

}  // namespace iad::sim
