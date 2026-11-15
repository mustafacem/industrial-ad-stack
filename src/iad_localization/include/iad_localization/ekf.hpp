// Planar EKF for a heavy vehicle: state [x, y, yaw, gyro_bias, odom_scale]. ROS-free.
//
// Prediction: wheel speed (scaled by the estimated odom scale, which also soaks up steady slip) and the
// bias-corrected gyro. Updates: GNSS antenna position (lever arm through yaw) and NDT pose, both
// Mahalanobis-gated, so multipath jumps and bad scan matches are rejected rather than averaged in.
// Measurements arrive late (NDT takes ms, GNSS has latency): the filter keeps a short history and
// re-applies inputs after inserting a measurement at its own timestamp.
#pragma once
#include <Eigen/Dense>
#include <deque>

#include "iad_localization/util.hpp"

namespace iad::localization {

class Ekf {
 public:
  using Vec5 = Eigen::Matrix<double, 5, 1>;
  using Mat5 = Eigen::Matrix<double, 5, 5>;
  struct Params {
    // Wheel speed: white noise (absolute + relative). Slip is not white: it is a correlated speed error
    // (mud: 15-30 % for seconds), carried by the odometry-scale state. Its random walk must be fast enough
    // to follow the ground, or the filter becomes overconfident and gates out good GNSS / NDT fixes.
    double sigma_v = 0.05, sigma_v_rel = 0.03;
    double sigma_w = 0.004;                     // gyro white noise [rad/s]
    double rw_bias = 2e-5, rw_scale = 0.04;     // random walks per sqrt(s)
    double gate_gnss = 9.21, gate_ndt = 11.34;  // chi2 99 % (2 and 3 DoF)
    bool gate = true;
    double gnss_inflate = 1.0;
    int reacquire_after = 30;  // consecutive GNSS rejections before the position covariance is re-opened
  };
  struct Stats { int gnss_used = 0, gnss_rejected = 0, ndt_used = 0, ndt_rejected = 0, reacquired = 0; };

  Ekf() = default;
  explicit Ekf(const Params& p) : p_(p) {}

  void init(double t, double x, double y, double yaw, double std_xy = 0.5, double std_yaw = 0.05) {
    x_ << x, y, yaw, 0.0, 1.0;
    P_.setZero();
    P_.diagonal() << sq(std_xy), sq(std_xy), sq(std_yaw), sq(0.005), sq(0.02);
    t_ = t, hist_.clear(), init_ = true;
  }
  bool initialized() const { return init_; }

  // Input at time t (IMU rate): wheel speed v [m/s], gyro z w [rad/s].
  void predict(double t, double v, double w) {
    if (!init_) return;
    const double dt = std::clamp(t - t_, 0.0, 0.1);
    hist_.push_back({t, v, w, x_, P_});
    while (!hist_.empty() && t - hist_.front().t > 3.0) hist_.pop_front();
    step(dt, v, w);
    t_ = t;
  }

  bool updateGnss(double t, const Eigen::Vector2d& z, const Eigen::Matrix2d& R, const Eigen::Vector2d& lever) {
    return delayed(t, [&] {
      const double c = std::cos(x_(2)), s = std::sin(x_(2));
      const Eigen::Vector2d h(x_(0) + c * lever.x() - s * lever.y(), x_(1) + s * lever.x() + c * lever.y());
      Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
      H(0, 0) = H(1, 1) = 1.0;
      H(0, 2) = -s * lever.x() - c * lever.y();
      H(1, 2) = c * lever.x() - s * lever.y();
      bool ok = correct<2>(z - h, H, p_.gnss_inflate * R, p_.gate_gnss);
      // Gating lock-out guard: a filter that has drifted rejects every fix forever. After a run of
      // rejections, admit that we are lost (inflate P) so the next fix can pull us back.
      rejected_run_ = ok ? 0 : rejected_run_ + 1;
      const bool other_source = t - last_pose_t_ < 2.0;  // NDT is holding us: the GNSS is what's wrong
      if (!ok && !other_source && p_.reacquire_after > 0 && rejected_run_ >= p_.reacquire_after) {
        P_(0, 0) += 25.0, P_(1, 1) += 25.0, P_(2, 2) += 0.02, rejected_run_ = 0, ++st_.reacquired;
        ok = correct<2>(z - h, H, p_.gnss_inflate * R, p_.gate_gnss);
      }
      ok ? ++st_.gnss_used : ++st_.gnss_rejected;
      return ok;
    });
  }

  bool updatePose(double t, const Eigen::Vector3d& z, const Eigen::Matrix3d& R) {
    return delayed(t, [&] {
      Eigen::Matrix<double, 3, 5> H = Eigen::Matrix<double, 3, 5>::Zero();
      H(0, 0) = H(1, 1) = H(2, 2) = 1.0;
      Eigen::Vector3d r = z - x_.head<3>();
      r(2) = wrapAngle(r(2));
      const bool ok = correct<3>(r, H, R, p_.gate_ndt);
      ok ? ++st_.ndt_used : ++st_.ndt_rejected;
      if (ok) last_pose_t_ = t;
      return ok;
    });
  }

  const Vec5& x() const { return x_; }
  const Mat5& P() const { return P_; }
  double t() const { return t_; }
  const Stats& stats() const { return st_; }
  Params& params() { return p_; }

 private:
  struct Rec { double t, v, w; Vec5 x; Mat5 P; };

  void step(double dt, double v_wheel, double w_gyro) {
    const double v = x_(4) * v_wheel, w = w_gyro - x_(3), c = std::cos(x_(2)), s = std::sin(x_(2));
    x_(0) += v * c * dt, x_(1) += v * s * dt, x_(2) = wrapAngle(x_(2) + w * dt);
    Mat5 F = Mat5::Identity();
    F(0, 2) = -v * s * dt, F(0, 4) = v_wheel * c * dt;
    F(1, 2) = v * c * dt, F(1, 4) = v_wheel * s * dt;
    F(2, 3) = -dt;
    Mat5 Q = Mat5::Zero();
    const double sv2 = sq(p_.sigma_v) + sq(p_.sigma_v_rel * v_wheel);
    Q(0, 0) = sv2 * c * c * dt * dt + 1e-6 * dt, Q(1, 1) = sv2 * s * s * dt * dt + 1e-6 * dt;
    Q(0, 1) = Q(1, 0) = sv2 * c * s * dt * dt;
    Q(2, 2) = sq(p_.sigma_w) * dt, Q(3, 3) = sq(p_.rw_bias) * dt, Q(4, 4) = sq(p_.rw_scale) * dt;
    P_ = F * P_ * F.transpose() + Q;
  }

  template <int N>
  bool correct(const Eigen::Matrix<double, N, 1>& r, const Eigen::Matrix<double, N, 5>& H,
               const Eigen::Matrix<double, N, N>& R, double gate) {
    const Eigen::Matrix<double, N, N> S = H * P_ * H.transpose() + R;
    const double d2 = r.dot(S.ldlt().solve(r));
    last_d2_ = d2;
    if (p_.gate && d2 > gate) return false;
    const Eigen::Matrix<double, 5, N> K = P_ * H.transpose() * S.inverse();
    x_ += K * r;
    x_(2) = wrapAngle(x_(2));
    const Mat5 I_KH = Mat5::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();  // Joseph form
    return true;
  }

  // Rewind to the measurement time, apply the update, replay the inputs that came after it.
  template <class F>
  bool delayed(double t, F&& update) {
    if (!init_) return false;
    size_t k = hist_.size();
    while (k > 0 && hist_[k - 1].t > t) --k;
    if (k == hist_.size()) return update();  // not in the past
    if (k == 0) return false;                 // older than the buffer
    const double t_now = t_;
    x_ = hist_[k].x, P_ = hist_[k].P;  // state just before input k (time ~ t)
    const bool ok = update();
    std::deque<Rec> replay(hist_.begin() + long(k), hist_.end());
    hist_.erase(hist_.begin() + long(k), hist_.end());
    double tl = hist_.empty() ? t : hist_.back().t;
    for (auto& r : replay) {
      hist_.push_back({r.t, r.v, r.w, x_, P_});
      step(std::clamp(r.t - tl, 0.0, 0.1), r.v, r.w);
      tl = r.t;
    }
    t_ = t_now;
    return ok;
  }

  Params p_;
  Vec5 x_ = Vec5::Zero();
  Mat5 P_ = Mat5::Identity();
  double t_ = 0, last_d2_ = 0;
  int rejected_run_ = 0;
  double last_pose_t_ = -1e9;
  bool init_ = false;
  std::deque<Rec> hist_;
  Stats st_;
};

}  // namespace iad::localization
