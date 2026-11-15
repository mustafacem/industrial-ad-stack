// controller_node: tracks /planning/trajectory with a delay-compensated linear MPC (lateral) + PI with
// speed/acceleration/grade feed-forward (longitudinal), or with the pure-pursuit baseline.
// Handles gear changes: finish the segment, stop, steer at standstill, then drive the next segment.
//
// In : /planning/trajectory (autoware_planning_msgs/Trajectory, v < 0 = reverse),
//      /localization/kinematic_state (Odometry), /vehicle/status/velocity (TwistWithCovarianceStamped),
//      /vehicle/status/steering (autoware_vehicle_msgs/SteeringReport)
// Out: /control/command (ackermann_msgs/AckermannDriveStamped: steering_angle, acceleration along the gear,
//      sign(speed) = gear), /control/predicted, /control/trail, /control/metrics (JSON)
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>

#include "iad_control/tracking.hpp"

namespace iad::control {

class ControllerNode : public rclcpp::Node {
 public:
  explicit ControllerNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("controller", opt) {
    algo_ = declare_parameter("algorithm", "mpc");
    vp_.wheelbase = declare_parameter("vehicle.wheelbase", 4.3), vp_.max_steer = declare_parameter("vehicle.max_steer", 0.6);
    vp_.max_steer_rate = declare_parameter("vehicle.max_steer_rate", 0.35), vp_.steer_tau = declare_parameter("vehicle.steer_tau", 0.25);
    vp_.delay = declare_parameter("vehicle.delay", 0.4), vp_.accel_tau = declare_parameter("vehicle.accel_tau", 0.6);
    vp_.max_acc = declare_parameter("vehicle.max_acc", 2.0), vp_.max_dec = declare_parameter("vehicle.max_dec", 2.5);
    comp_ = declare_parameter("delay_compensation", true);
    grade_ff_ = declare_parameter("grade_feedforward", true);
    ld_min_ = declare_parameter("lookahead_min", 6.0);
    ld_gain_ = declare_parameter("lookahead_gain", 1.2);
    kp_ = declare_parameter("kp_speed", 0.6);
    ki_ = declare_parameter("ki_speed", 0.25);
    rr_ff_ = declare_parameter("rolling_resistance_ff", 0.03);  // fraction of g, nominal haul-road value
    MpcParams mp;
    mp.delay_comp = comp_;
    mpc_ = std::make_unique<LateralMpc>(vp_, mp);
    mpc_dt_ = mp.dt, mpc_n_ = mp.N;
    hist_len_ = size_t(std::lround(vp_.delay / kDt));

    cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/command", 10);
    pred_pub_ = create_publisher<nav_msgs::msg::Path>("/control/predicted", 5);
    trail_pub_ = create_publisher<nav_msgs::msg::Path>("/control/trail", 2);
    ref_pub_ = create_publisher<nav_msgs::msg::Path>("/control/reference", rclcpp::QoS(1).transient_local().reliable());
    metrics_pub_ = create_publisher<std_msgs::msg::String>("/control/metrics", 10);
    traj_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>("/planning/trajectory", rclcpp::QoS(1).transient_local().reliable(),
        [this](autoware_planning_msgs::msg::Trajectory::ConstSharedPtr m) { onTrajectory(*m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", "/localization/kinematic_state"), 20,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& q = m->pose.pose.orientation;
          const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
          s_.x = m->pose.pose.position.x, s_.y = m->pose.pose.position.y, s_.yaw = rpy.z(), s_.pitch = rpy.y();
          have_odom_ = true;
        });
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/status/velocity", 20,
        [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) { s_.v = m->twist.twist.linear.x; });
    steer_sub_ = create_subscription<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering", 20,
        [this](autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr m) { s_.steer = m->steering_tire_angle; });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(int(kDt * 1000)), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "controller: %s, delay comp %s, grade ff %s, actuator delay %.2f s", algo_.c_str(),
                comp_ ? "on" : "off", grade_ff_ ? "on" : "off", vp_.delay);
  }

 private:
  static constexpr double kDt = 0.05;

  void onTrajectory(const autoware_planning_msgs::msg::Trajectory& m) {
    path_.clear(), z_.clear();
    for (const auto& p : m.points) {
      const auto& q = p.pose.orientation;
      RefPoint r;
      r.x = p.pose.position.x, r.y = p.pose.position.y;
      r.yaw = rpyFromQuat(Eigen::Quaterniond(q.w, q.x, q.y, q.z)).z();
      r.v = p.longitudinal_velocity_mps, r.a = p.acceleration_mps2;
      r.kappa = std::tan(p.front_wheel_angle_rad) / vp_.wheelbase;  // steering curvature; geometric below
      path_.push_back(r), z_.push_back(p.pose.position.z);
    }
    inferDirections(path_);
    for (auto& r : path_) r.kappa *= r.dir;  // geometric curvature dyaw/ds (what the MPC linearises about)
    nav_msgs::msg::Path ref;
    ref.header = m.header;
    for (size_t i = 0; i < path_.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = m.header, ps.pose.position.x = path_[i].x, ps.pose.position.y = path_[i].y, ps.pose.position.z = z_[i] + 0.3;
      ps.pose.orientation.z = std::sin(path_[i].yaw / 2), ps.pose.orientation.w = std::cos(path_[i].yaw / 2);
      ref.poses.push_back(ps);
    }
    ref_pub_->publish(ref);
    segs_.clear(), seg_ = 0, near_ = 0, done_ = path_.size() < 2, integ_ = 0, dock_set_ = false;
    size_t b = 0;
    for (size_t i = 2; i < path_.size(); ++i)
      if (path_[i].dir != path_[i - 1].dir) segs_.push_back({b, i - 1}), b = i - 1;
    if (path_.size() >= 2) segs_.push_back({b, path_.size() - 1});
    // Re-synchronise: an updated trajectory (stop line, resume, stitched replan) may still contain the part
    // already driven. Start on the segment nearest to the vehicle whose gear matches the current motion.
    if (have_odom_ && segs_.size() > 1) {
      double best = 1e18;
      for (size_t k = 0; k < segs_.size(); ++k) {
        const int dir = path_[std::min(segs_[k].first + 1, segs_[k].second)].dir;
        if (std::abs(s_.v) > 0.2 && dir * s_.v < 0) continue;
        for (size_t i = segs_[k].first; i <= segs_[k].second; ++i) {
          const double d = std::hypot(path_[i].x - s_.x, path_[i].y - s_.y);
          if (d < best) best = d, seg_ = k, near_ = i;
        }
      }
    }
  }

  // Reference point at arc length s (linear interpolation).
  RefPoint at(double s) const {
    if (s <= path_.front().s) return path_.front();
    for (size_t i = 1; i < path_.size(); ++i)
      if (path_[i].s >= s) {
        const auto &a = path_[i - 1], &b = path_[i];
        const double u = (s - a.s) / std::max(1e-6, b.s - a.s);
        RefPoint r = a;
        r.x += u * (b.x - a.x), r.y += u * (b.y - a.y), r.yaw = wrapAngle(a.yaw + u * wrapAngle(b.yaw - a.yaw));
        r.v += u * (b.v - a.v), r.kappa += u * (b.kappa - a.kappa), r.s = s;
        return r;
      }
    return path_.back();
  }

  void tick() {
    if (!have_odom_) return;
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now(), cmd.header.frame_id = "base_link";
    updateTrail();
    if (done_ || path_.empty()) {
      if (done_ && !path_.empty() && !dock_set_) {  // docking error vs the final trajectory pose
        const auto& g = path_.back();
        const double dx = s_.x - g.x, dy = s_.y - g.y;
        dock_long_ = dx * std::cos(g.yaw) + dy * std::sin(g.yaw), dock_lat_ = -dx * std::sin(g.yaw) + dy * std::cos(g.yaw);
        dock_head_ = rad2deg(wrapAngle(s_.yaw - g.yaw)), dock_set_ = true, dock_t_ = now().seconds();
      }
      cmd.drive.steering_angle = float(s_.steer), cmd.drive.acceleration = -1.5f;
      cmd_pub_->publish(cmd);
      state_ = path_.empty() ? "idle" : "done";
      push(s_.steer);
      publishMetrics(0, 0, 0, 0, 0, 0);
      return;
    }
    const auto [b, e] = segs_[seg_];
    const int dir = path_[std::min(b + 1, e)].dir;
    const Errors ea = trackingErrors(path_, b, e, s_.x, s_.y, s_.yaw, std::max(near_, b));
    near_ = ea.idx;
    // Segment end: stopped at (or a bit past) its last point -> next segment (gear change) or done.
    const double d_end = std::hypot(s_.x - path_[e].x, s_.y - path_[e].y);
    if (std::abs(s_.v) < 0.08 && (d_end < 0.25 || progress(near_, b, e) > path_[e].s - 0.15)) {
      if (++seg_ >= segs_.size()) {
        done_ = true;
        return;
      }
      near_ = segs_[seg_].first, integ_ = 0;
      return;
    }
    // Lateral
    Stopwatch sw;
    State sp = comp_ ? predictState(s_, hist_, kDt, vp_) : s_;
    const Errors ep = trackingErrors(path_, b, e, sp.x, sp.y, sp.yaw, near_);
    double u, pred_lat = 0;
    int iters = 0;
    nav_msgs::msg::Path pred;
    pred.header.frame_id = "map", pred.header.stamp = now();
    if (algo_ == "mpc") {
      std::vector<double> vs, ks;
      std::vector<double> ss;
      double sk = path_[ep.idx].s;
      const double s_end = path_[e].s;
      for (int k = 0; k < mpc_n_; ++k) {
        const RefPoint r = at(std::min(sk, s_end));
        const double vm = std::max(0.5, 0.5 * (std::abs(s_.v) + std::abs(r.v)));
        vs.push_back(dir * vm), ks.push_back(r.kappa), ss.push_back(sk);
        sk += vm * mpc_dt_;
      }
      std::vector<Eigen::Vector3d> X;
      u = mpc_->solve(Eigen::Vector3d(ep.lat, ep.head, sp.steer), vs, ks, last_u_, &X, &iters);
      for (size_t k = 0; k < X.size(); ++k) {
        const RefPoint r = at(std::min(ss[k], s_end));
        geometry_msgs::msg::PoseStamped ps;
        ps.header = pred.header;
        ps.pose.position.x = r.x - X[k](0) * std::sin(r.yaw), ps.pose.position.y = r.y + X[k](0) * std::cos(r.yaw);
        ps.pose.position.z = zAt(r.s) + 0.8, ps.pose.orientation.w = 1;
        pred.poses.push_back(ps);
      }
      if (!X.empty()) pred_lat = X.back()(0);
    } else {
      const double ld = std::max(ld_min_, ld_gain_ * std::abs(s_.v));
      double tx, ty;
      u = purePursuit(path_, b, e, ep.idx, sp, ld, vp_, &tx, &ty);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = pred.header, ps.pose.orientation.w = 1;
      ps.pose.position.x = sp.x, ps.pose.position.y = sp.y, ps.pose.position.z = zAt(path_[ep.idx].s) + 0.8;
      pred.poses.push_back(ps);
      ps.pose.position.x = tx, ps.pose.position.y = ty;
      pred.poses.push_back(ps);
    }
    const double solve_ms = sw.ms();
    // Longitudinal: reference where the truck will be when this command lands; stop at the segment end.
    // Continuous progress along the segment: the nearest index plus the projection on its tangent (the
    // index alone quantises the remaining distance by a whole point spacing and stops the truck short).
    const double s_now = progress(near_, b, e);
    const double t_ahead = comp_ ? vp_.delay + vp_.accel_tau : 0.0;
    const double s_look = std::min(s_now + std::abs(s_.v) * t_ahead, path_[e].s);
    const RefPoint r = at(s_look);
    const double to_go = path_[e].s - s_now;
    // Distance still to brake in: what the truck covers before a new command takes effect (dead time + drivetrain
    // lag) is already spent.
    const double rem = to_go - (comp_ ? std::abs(s_.v) * (vp_.delay + vp_.accel_tau) : 0.0);
    // Three regimes: (1) stop line - the plan is zero from here to the segment end: hold; (2) final approach,
    // the last 2 m: a stopping law on the remaining distance with a creep floor (the plan's own end ramp, read
    // `t_ahead` early, would stop the truck short); (3) elsewhere: the planned speed.
    bool plan_stopped = true;
    for (size_t i = std::max(near_, b); i < e && plan_stopped; ++i) plan_stopped = std::abs(path_[i].v) < 0.05;
    const double v_stop = std::sqrt(2.0 * 0.4 * std::max(0.0, rem - 0.05));
    double v_des;
    if (plan_stopped && to_go > 0.3) {
      v_des = 0.0;
    } else if (to_go < 2.0) {
      v_des = std::min({std::max(std::abs(at(s_now).v), 0.3), v_stop, 0.5});  // docking speed
      if (to_go > 0.5) v_des = std::max(v_des, 0.3);
      integ_ = std::min(integ_, 0.0);  // no wound-up push into the dock
    } else {
      v_des = std::min(std::abs(r.v), v_stop);
      if (std::abs(r.v) > 0.05) v_des = std::max(v_des, 0.3);  // never stall on a near-zero ramp of the plan
    }
    const double ev = v_des - std::abs(s_.v);
    integ_ = std::clamp(integ_ + ev * kDt, -4.0, 4.0);
    // Feed-forward: planned acceleration (only once rolling, it would cancel the breakaway effort), rolling
    // resistance, and the grade (gravity along the body axis, whichever gear).
    const double a_ff = (comp_ && std::abs(s_.v) > 0.8 ? r.a : 0.0) + (v_des > 0.1 ? rr_ff_ * kGravity : 0.0);
    // `a` is the acceleration ALONG THE GEAR (> 0 speeds up in the selected direction, < 0 brakes), which is
    // what /control/command carries; gravity along the body x axis maps into it with the gear sign.
    double a = a_ff + kp_ * ev + ki_ * integ_;
    if (grade_ff_) a -= dir * kGravity * std::sin(s_.pitch);
    if (to_go < 2.0 && std::abs(s_.v) > v_des + 0.05)  // brake to land on the end: v^2 / (2 * distance left)
      a = std::min(a, -(s_.v * s_.v - v_des * v_des) / (2.0 * std::max(0.15, rem)));
    if (v_des > 0.2 && std::abs(s_.v) < 0.1 && a < 0.8) a = 0.8;  // breakaway from standstill
    if (near_ <= b + 2 && std::abs(s_.v) < 0.05 && std::abs(u - s_.steer) > 0.12) a = -1.0;  // steer at standstill first
    a = std::clamp(a, -vp_.max_dec, vp_.max_acc);
    cmd.drive.steering_angle = float(u), cmd.drive.acceleration = float(a), cmd.drive.speed = float(dir * v_des);
    cmd_pub_->publish(cmd);
    push(u);
    pred_pub_->publish(pred);
    state_ = dir > 0 ? "forward" : "reverse";
    to_go_ = to_go;
    publishMetrics(ea.lat, ea.head, v_des, a, u, solve_ms, iters, pred_lat);
  }

  double progress(size_t i, size_t b, size_t e) const {
    const size_t j = std::min(std::max(i, b) + 1, e), k = j - 1;
    const double tx = path_[j].x - path_[k].x, ty = path_[j].y - path_[k].y, L = std::hypot(tx, ty);
    if (L < 1e-6) return path_[k].s;
    const double u = ((s_.x - path_[k].x) * tx + (s_.y - path_[k].y) * ty) / L;
    return path_[k].s + std::clamp(u, -L, 2.0 * L);
  }

  double zAt(double s) const {
    if (z_.empty()) return 0.0;
    for (size_t i = 1; i < path_.size(); ++i)
      if (path_[i].s >= s) return z_[i];
    return z_.back();
  }

  void updateTrail() {
    if (!trail_.poses.empty()) {
      const auto& l = trail_.poses.back().pose.position;
      if (std::hypot(l.x - s_.x, l.y - s_.y) < 0.5) return;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map", ps.pose.orientation.w = 1;
    ps.pose.position.x = s_.x, ps.pose.position.y = s_.y, ps.pose.position.z = (path_.empty() ? 0.0 : zAt(path_[near_].s)) + 0.6;
    trail_.poses.push_back(ps);
    trail_.header.frame_id = "map", trail_.header.stamp = now();
    trail_pub_->publish(trail_);
  }

  void push(double u) {
    hist_.push_back(u), last_u_ = u;
    while (hist_.size() > hist_len_) hist_.pop_front();
  }

  void publishMetrics(double lat, double head, double v_des, double a, double u, double ms, int iters = 0, double pred_lat = 0) {
    if (std::abs(s_.v) > 0.5 && !done_) {
      n_ += 1, sum2_ += lat * lat, max_ = std::max(max_, std::abs(lat)), sumh2_ += head * head;
    }
    std::ostringstream js;
    js.precision(4);
    js << std::fixed << "{\"t\":" << now().seconds() << ",\"algorithm\":\"" << algo_ << "\",\"delay_comp\":" << (comp_ ? "true" : "false")
       << ",\"state\":\"" << state_ << "\",\"lat_err\":" << lat << ",\"head_err_deg\":" << rad2deg(head) << ",\"v\":" << s_.v
       << ",\"v_des\":" << v_des << ",\"a_cmd\":" << a << ",\"steer_cmd\":" << u << ",\"steer\":" << s_.steer
       << ",\"solve_ms\":" << ms << ",\"qp_iters\":" << iters << ",\"pred_lat_end\":" << pred_lat
       << ",\"to_go\":" << to_go_ << ",\"segment\":" << seg_ << ",\"segments\":" << segs_.size() << ",\"lat_rms\":" << std::sqrt(sum2_ / std::max(1.0, n_))
       << ",\"lat_max\":" << max_ << ",\"head_rms_deg\":" << rad2deg(std::sqrt(sumh2_ / std::max(1.0, n_)))
       << ",\"docked\":" << (dock_set_ ? "true" : "false") << ",\"dock_long\":" << dock_long_ << ",\"dock_lat\":" << dock_lat_
       << ",\"dock_head_deg\":" << dock_head_ << ",\"dock_t\":" << dock_t_ << "}";
    std_msgs::msg::String s;
    s.data = js.str();
    metrics_pub_->publish(s);
  }

  std::string algo_, state_ = "idle";
  VehicleParams vp_;
  bool comp_ = true, grade_ff_ = true, have_odom_ = false, done_ = true;
  double ld_min_, ld_gain_, kp_, ki_, rr_ff_, integ_ = 0, last_u_ = 0, mpc_dt_ = 0.1;
  int mpc_n_ = 30;
  std::unique_ptr<LateralMpc> mpc_;
  RefPath path_;
  std::vector<double> z_;
  std::vector<std::pair<size_t, size_t>> segs_;
  size_t seg_ = 0, near_ = 0, hist_len_ = 8;
  std::deque<double> hist_;
  State s_;
  double to_go_ = 0, n_ = 0, sum2_ = 0, max_ = 0, sumh2_ = 0, dock_long_ = 0, dock_lat_ = 0, dock_head_ = 0, dock_t_ = 0;
  bool dock_set_ = false;
  nav_msgs::msg::Path trail_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_pub_, trail_pub_, ref_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace iad::control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iad::control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
