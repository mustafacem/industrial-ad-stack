// iad_gz_sim::VehicleDrive: drive-by-wire for the wheeled truck models (gz-sim system + embedded
// rclcpp node, the gz_ros2_control pattern).
//
//   /control/command (ackermann_msgs/AckermannDriveStamped)
//       steering_angle = road-wheel angle [rad], acceleration [m/s^2] along the selected gear,
//       sign(speed) = gear (+ forward, - reverse; 0 keeps the current gear)
//   -> actuator model: dead time, steering first-order lag + rate limit, drivetrain lag,
//      power and traction limits, service brakes, hill hold, surface rolling resistance
//   -> wheel joint torques (rear-wheel drive, brakes on all wheels) and Ackermann steering-joint servos.
//
// Publishes, stamped with sim time:
//   /vehicle/status/velocity   geometry_msgs/TwistWithCovarianceStamped  wheel odometry (scale error, noise)
//   /vehicle/status/steering   autoware_vehicle_msgs/SteeringReport      measured road-wheel angle
//   /sim/ground_truth/odom     nav_msgs/Odometry  (map -> base_link; optional TF for stand-in localization)
//   /sim/vehicle/debug         std_msgs/String JSON (applied steer/accel/torques/surface, for evaluation)
#include <gz/msgs/particle_emitter.pb.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <deque>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

#include "iad_gz_sim/terrain_grid.hpp"

namespace iad_gz_sim {

using gz::sim::Entity;
using gz::sim::EntityComponentManager;

class VehicleDrive : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate {
 public:
  ~VehicleDrive() override {
    if (exec_) exec_->cancel();
    if (spin_.joinable()) spin_.join();
  }

  void Configure(const Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, EntityComponentManager& ecm,
                 gz::sim::EventManager&) override {
    model_ = gz::sim::Model(entity);
    auto num = [&](const char* k, double d) { return sdf->HasElement(k) ? sdf->Get<double>(k) : d; };
    auto str = [&](const char* k, const std::string& d) { return sdf->HasElement(k) ? sdf->Get<std::string>(k) : d; };
    L_ = num("wheelbase", 4.3), track_ = num("track", 3.3), R_ = num("wheel_radius", 1.2);
    nominal_mass_ = num("nominal_mass", 32000), max_steer_ = num("max_steer", 0.6);
    max_steer_rate_ = num("max_steer_rate", 0.35), steer_tau_ = num("steer_tau", 0.25), delay_ = num("delay", 0.4);
    accel_tau_ = num("accel_tau", 0.6), max_acc_ = num("max_acc", 2.0), max_dec_ = num("max_dec", 2.5);
    power_ = num("power", 400e3), rr_scale_ = num("rolling_resistance_scale", 1.0);
    odom_scale_err_ = num("odom_scale_error", 0.015), odom_noise_ = num("odom_noise", 0.02);
    gt_tf_ = num("publish_gt_tf", 0) > 0.5;
    map_frame_ = str("map_frame", "map"), base_frame_ = str("base_frame", "base_link");
    for (const char* k : {"rear_left_wheel", "rear_right_wheel", "front_left_wheel", "front_right_wheel",
                          "front_left_steer", "front_right_steer"}) {
      const Entity j = model_.JointByName(ecm, str(k, ""));
      if (j == gz::sim::kNullEntity) gzerr << "VehicleDrive: missing joint <" << k << ">\n";
      joints_[k] = gz::sim::Joint(j);
      joints_[k].EnablePositionCheck(ecm, true), joints_[k].EnableVelocityCheck(ecm, true);
    }
    chassis_ = gz::sim::Link(model_.LinkByName(ecm, str("chassis_link", "chassis")));
    chassis_.EnableVelocityChecks(ecm, true);
    mass_ = 0;
    for (const Entity l : model_.Links(ecm))
      if (auto in = ecm.Component<gz::sim::components::Inertial>(l)) mass_ += in->Data().MassMatrix().Mass();
    const std::string grid = str("terrain_grid", "");
    if (!grid.empty() && !terrain_.load(iad::gz::findResource(grid))) gzerr << "VehicleDrive: no terrain grid " << grid << "\n";
    dust_topic_ = str("dust_emitter_topic", "");
    if (!dust_topic_.empty()) dust_pub_ = gzn_.Advertise<gz::msgs::ParticleEmitter>(dust_topic_);

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("vehicle_drive", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
    cmd_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        str("command_topic", "/control/command"), 10, [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          latest_ = *m, have_new_ = true;
        });
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/status/velocity", 20);
    steer_pub_ = node_->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering", 20);
    gt_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/sim/ground_truth/odom", 20);
    dbg_pub_ = node_->create_publisher<std_msgs::msg::String>("/sim/vehicle/debug", 10);
    if (gt_tf_) tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spin_ = std::thread([this] { exec_->spin(); });
    gzmsg << "VehicleDrive: " << model_.Name(ecm) << " mass " << mass_ << " kg, L " << L_ << " m, delay " << delay_ << " s\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, EntityComponentManager& ecm) override {
    if (info.paused) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (have_new_) {
        pipe_.push_back({t, latest_.drive.steering_angle, latest_.drive.acceleration, latest_.drive.speed});
        have_new_ = false, last_cmd_t_ = t;
      }
    }
    // Dead time: a command acts `delay` seconds after it arrived.
    while (!pipe_.empty() && pipe_.front().t + delay_ <= t) {
      const Cmd c = pipe_.front();
      pipe_.pop_front();
      steer_cmd_ = std::clamp(c.steer, -max_steer_, max_steer_), acc_cmd_ = c.accel, active_ = true;
      if (c.speed > 1e-3) gear_ = 1;
      else if (c.speed < -1e-3) gear_ = -1;
    }
    const bool timeout = active_ && t - last_cmd_t_ > 0.5 + delay_;
    const double acc_req = !active_ || timeout ? -max_dec_ : acc_cmd_;

    // Steering: first-order lag with a rate limit (hydraulic steering), then Ackermann joint servos.
    const double rate = std::clamp((steer_cmd_ - steer_) / steer_tau_, -max_steer_rate_, max_steer_rate_);
    steer_ = std::clamp(steer_ + rate * dt, -max_steer_, max_steer_);
    double dl = steer_, dr = steer_;
    if (std::abs(steer_) > 1e-4) {
      const double Rt = L_ / std::tan(steer_);
      dl = std::atan(L_ / (Rt - track_ / 2)), dr = std::atan(L_ / (Rt + track_ / 2));
    }
    servo(ecm, "front_left_steer", dl), servo(ecm, "front_right_steer", dr);

    // Longitudinal: drivetrain / brake lag on the requested acceleration.
    acc_ += (acc_req - acc_) * std::min(1.0, dt / accel_tau_);
    const std::optional<gz::math::Pose3d> pose = gz::sim::worldPose(model_.Entity(), ecm);
    const auto vw = chassis_.WorldLinearVelocity(ecm);
    if (!pose || !vw) return;
    const double yaw = pose->Rot().Yaw();
    v_ = std::cos(yaw) * vw->X() + std::sin(yaw) * vw->Y();
    const double g = 9.81;
    const uint8_t surf = terrain_.material(pose->Pos().X(), pose->Pos().Y());
    surface_ = surf;
    const double crr = rr_scale_ * iad::gz::rollingResistance(surf);
    std::array<const char*, 4> wheels = {"rear_left_wheel", "rear_right_wheel", "front_left_wheel", "front_right_wheel"};
    const double along = acc_;  // command convention: acceleration along the selected gear (> 0 drives, < 0 brakes)
    // Hill hold at standstill unless the request is a real start (> 0.3 m/s^2 along the gear).
    // (A slow final creep with a small positive request keeps rolling; a stop request holds at once.)
    hold_ = (std::abs(v_) < 0.05 && acc_req <= 0.3) || (std::abs(v_) < 0.15 && acc_req <= 0.0) || !active_ || timeout;
    hold_ = hold_ && std::abs(v_) < 0.15;
    if (hold_) {
      // Parking brake: a stiff torsional spring-damper on each wheel about the angle where the hold engaged,
      // clamped to the brake capacity (static friction of the brake discs).
      const double t_max = 0.3 * mass_ * max_dec_ * R_;
      for (size_t k = 0; k < 4; ++k) {
        const auto p = joints_[wheels[k]].Position(ecm);
        const auto w = joints_[wheels[k]].Velocity(ecm);
        const double q = p && !p->empty() ? (*p)[0] : 0.0, om = w && !w->empty() ? (*w)[0] : 0.0;
        if (!was_hold_) hold_q_[k] = q;
        joints_[wheels[k]].SetForce(ecm, {std::clamp(-5e5 * (q - hold_q_[k]) - 2e5 * om, -t_max, t_max)});
      }
      was_hold_ = true, drive_torque_ = 0;
      return;
    }
    was_hold_ = false;
    // The "ECU" maps the request to wheel force with the nominal (empty) mass; the real mass, grade and
    // rolling resistance are left to physics, so the controller has to compensate for them.
    double drive = 0, brake = 0;
    if (along > 0) {
      const double F = std::min({nominal_mass_ * along, nominal_mass_ * max_acc_, power_ / std::max(1.0, std::abs(v_))});
      drive = F * gear_;
    } else {
      brake = nominal_mass_ * std::min(-along, max_dec_);
    }
    drive_torque_ = drive * R_;
    for (size_t k = 0; k < 4; ++k) {
      const auto w = joints_[wheels[k]].Velocity(ecm);
      const double om = w && !w->empty() ? (*w)[0] : 0.0;
      double T = k < 2 ? 0.5 * drive * R_ : 0.0;
      const double sgn = std::tanh(om / 0.4);
      T -= sgn * 0.25 * brake * R_;
      T -= sgn * crr * 0.25 * mass_ * g * R_;
      joints_[wheels[k]].SetForce(ecm, {T});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const EntityComponentManager& ecm) override {
    if (info.paused) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t - last_pub_ < 0.02 - 1e-9) return;  // 50 Hz
    last_pub_ = t;
    const std::optional<gz::math::Pose3d> pose = gz::sim::worldPose(model_.Entity(), ecm);
    const auto vw = chassis_.WorldLinearVelocity(ecm);
    const auto ww = chassis_.WorldAngularVelocity(ecm);
    if (!pose || !vw || !ww) return;
    builtin_interfaces::msg::Time st;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    st.sec = int32_t(ns / 1000000000), st.nanosec = uint32_t(ns % 1000000000);

    // Wheel odometry: mean rear-wheel speed with a tyre-radius scale error; yaw rate from the steering.
    auto jv = [&](const char* n) {
      const auto v = joints_.at(n).Velocity(ecm);
      return v && !v->empty() ? (*v)[0] : 0.0;
    };
    auto jp = [&](const char* n) {
      const auto v = joints_.at(n).Position(ecm);
      return v && !v->empty() ? (*v)[0] : 0.0;
    };
    const double v_wheel = 0.5 * (jv("rear_left_wheel") + jv("rear_right_wheel")) * R_ * (1.0 + odom_scale_err_) +
                           odom_noise_ * nrm_(rng_);
    const double tl = std::tan(jp("front_left_steer")), tr = std::tan(jp("front_right_steer"));
    const double steer_meas = std::abs(tl + tr) > 1e-6 ? std::atan(2.0 * tl * tr / (tl + tr)) : 0.0;
    geometry_msgs::msg::TwistWithCovarianceStamped tw;
    tw.header.stamp = st, tw.header.frame_id = base_frame_;
    tw.twist.twist.linear.x = v_wheel;
    tw.twist.twist.angular.z = v_wheel * std::tan(steer_meas) / L_;
    tw.twist.covariance[0] = std::pow(0.05 + 0.02 * std::abs(v_wheel), 2);
    tw.twist.covariance[35] = std::pow(0.02, 2);
    vel_pub_->publish(tw);
    autoware_vehicle_msgs::msg::SteeringReport sr;
    sr.stamp = st, sr.steering_tire_angle = float(steer_meas);
    steer_pub_->publish(sr);

    const auto& q = pose->Rot();
    const gz::math::Vector3d vb = q.RotateVectorReverse(*vw);
    const gz::math::Vector3d wb = q.RotateVectorReverse(*ww);
    nav_msgs::msg::Odometry od;
    od.header.stamp = st, od.header.frame_id = map_frame_, od.child_frame_id = base_frame_;
    od.pose.pose.position.x = pose->Pos().X(), od.pose.pose.position.y = pose->Pos().Y(), od.pose.pose.position.z = pose->Pos().Z();
    od.pose.pose.orientation.w = q.W(), od.pose.pose.orientation.x = q.X(), od.pose.pose.orientation.y = q.Y(), od.pose.pose.orientation.z = q.Z();
    od.twist.twist.linear.x = vb.X(), od.twist.twist.linear.y = vb.Y(), od.twist.twist.linear.z = vb.Z();
    od.twist.twist.angular.x = wb.X(), od.twist.twist.angular.y = wb.Y(), od.twist.twist.angular.z = wb.Z();
    gt_pub_->publish(od);
    if (tfb_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = od.header, tf.child_frame_id = base_frame_;
      tf.transform.translation.x = od.pose.pose.position.x, tf.transform.translation.y = od.pose.pose.position.y;
      tf.transform.translation.z = od.pose.pose.position.z, tf.transform.rotation = od.pose.pose.orientation;
      tfb_->sendTransform(tf);
    }
    if (++dbg_div_ % 5 == 0) {
      std_msgs::msg::String d;
      char buf[384];
      std::snprintf(buf, sizeof(buf),
                    "{\"t\":%.3f,\"steer\":%.4f,\"steer_cmd\":%.4f,\"acc\":%.3f,\"gear\":%d,\"v\":%.3f,\"v_wheel\":%.3f,"
                    "\"hold\":%d,\"surface\":%d,\"drive_torque\":%.0f,\"pitch\":%.4f,\"roll\":%.4f,\"mass\":%.0f}",
                    t, steer_, steer_cmd_, acc_, gear_, v_, v_wheel, hold_ ? 1 : 0, int(surface_), drive_torque_,
                    pose->Rot().Pitch(), pose->Rot().Roll(), mass_);
      d.data = buf;
      dbg_pub_->publish(d);
    }
    if (!dust_topic_.empty()) {
      const bool loose = surface_ == iad::gz::kDirt || surface_ == iad::gz::kGravel || surface_ == iad::gz::kMud;
      const bool on = loose && std::abs(v_) > 2.5;
      if (on != dust_on_) {
        gz::msgs::ParticleEmitter m;
        m.mutable_emitting()->set_data(on);
        dust_pub_.Publish(m);
        dust_on_ = on;
      }
    }
  }

 private:
  struct Cmd {
    double t, steer, accel, speed;
  };
  void servo(EntityComponentManager& ecm, const char* j, double target) {
    const auto p = joints_[j].Position(ecm);
    const double q = p && !p->empty() ? (*p)[0] : 0.0;
    joints_[j].SetVelocity(ecm, {std::clamp(25.0 * (target - q), -2.0, 2.0)});
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link chassis_{gz::sim::kNullEntity};
  std::map<std::string, gz::sim::Joint> joints_;
  double L_ = 4.3, track_ = 3.3, R_ = 1.2, nominal_mass_ = 32000, mass_ = 32000, max_steer_ = 0.6, max_steer_rate_ = 0.35;
  double steer_tau_ = 0.25, delay_ = 0.4, accel_tau_ = 0.6, max_acc_ = 2, max_dec_ = 2.5, power_ = 400e3, rr_scale_ = 1;
  double odom_scale_err_ = 0.015, odom_noise_ = 0.02;
  double steer_ = 0, steer_cmd_ = 0, acc_cmd_ = 0, acc_ = 0, v_ = 0, last_cmd_t_ = -1, last_pub_ = -1, drive_torque_ = 0;
  int gear_ = 1, dbg_div_ = 0;
  bool active_ = false, hold_ = true, was_hold_ = false, gt_tf_ = false, dust_on_ = false;
  std::array<double, 4> hold_q_{};
  uint8_t surface_ = 0;
  std::deque<Cmd> pipe_;
  std::string map_frame_, base_frame_, dust_topic_;
  iad::gz::TerrainGrid terrain_;
  std::mt19937 rng_{42};
  std::normal_distribution<double> nrm_{0.0, 1.0};

  std::mutex mtx_;
  ackermann_msgs::msg::AckermannDriveStamped latest_;
  bool have_new_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dbg_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  gz::transport::Node gzn_;
  gz::transport::Node::Publisher dust_pub_;
};

}  // namespace iad_gz_sim

GZ_ADD_PLUGIN(iad_gz_sim::VehicleDrive, gz::sim::System, iad_gz_sim::VehicleDrive::ISystemConfigure,
              iad_gz_sim::VehicleDrive::ISystemPreUpdate, iad_gz_sim::VehicleDrive::ISystemPostUpdate)
