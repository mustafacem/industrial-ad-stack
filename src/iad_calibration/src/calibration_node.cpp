// calibration_node: target-less extrinsic calibration of the corner LiDARs against the roof LiDAR.
//   ground    parked on flat ground: per-LiDAR ground plane -> roll, pitch, height
//   collect   driving: keyframes of every corner LiDAR + a reference map from the roof LiDAR
//   register  joint Gauss-Newton per LiDAR over all keyframes -> yaw, x, y (one visible step per tick)
//   refine    joint damped Gauss-Newton -> all six extrinsic DOFs, including residual ground-fit error
//   done      calibrated extrinsics written to YAML
// The simulator's true extrinsics (gt/ frames) are read only to report the error before/after.
#include <fstream>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "iad_calibration/calibration.hpp"

namespace iad::calibration {

struct Sensor {
  std::string name, frame;
  Eigen::Isometry3d T_nom, T_est;
  bool have_tf = false;
  std::vector<Eigen::Vector3d> latest;
  std::vector<Eigen::Vector3d> ground_pts;
  int ground_scans = 0;
  std::vector<Keyframe> kfs;
  double rms = 0;
  int matched = 0;
};

class CalibrationNode : public rclcpp::Node {
 public:
  explicit CalibrationNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("calibration", opt) {
    const auto names = declare_parameter("lidars", std::vector<std::string>{"front_left", "front_right", "rear_left", "rear_right"});
    ref_name_ = declare_parameter("reference", "top");
    spacing_ = declare_parameter("keyframe_spacing", 2.5);
    max_kf_ = int(declare_parameter("max_keyframes", 40));
    n_ground_ = int(declare_parameter("ground_scans", 10));
    max_iter_ = int(declare_parameter("max_iterations", 25));
    max_iter_6dof_ = int(declare_parameter("max_iterations_6dof", 15));
    yaml_ = declare_parameter("output", "");  // YAML file for the result (empty = metrics topic only)
    front_ = declare_parameter("vehicle.front", 10.0), rear_ = declare_parameter("vehicle.rear", 3.0);
    width_ = declare_parameter("vehicle.width", 3.0), height_ = declare_parameter("vehicle.height", 4.2);
    const double period = declare_parameter("iteration_period", 0.35);
    for (const auto& n : names) sensors_[n] = Sensor{n, "lidar_" + n, {}, {}, false, {}, {}, 0, {}, 0, 0};
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    metrics_pub_ = create_publisher<std_msgs::msg::String>("/calibration/metrics", 10);
    ref_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/calibration/reference", rclcpp::QoS(1).transient_local().reliable());
    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    for (auto& [n, s] : sensors_) {
      cloud_pubs_[n] = create_publisher<sensor_msgs::msg::PointCloud2>("/calibration/clouds/" + n, 2);
      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>("/sensing/lidar/" + n + "/points", qos,
          [this, n](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { sensors_[n].latest = readXyz(*m); }));
    }
    subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>("/sensing/lidar/" + ref_name_ + "/points", qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { ref_latest_ = readXyz(*m), ref_frame_ = m->header.frame_id; }));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", "/localization/kinematic_state"), 20,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          T_mb_ = Eigen::Isometry3d::Identity();
          const auto& q = m->pose.pose.orientation;
          T_mb_.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
          T_mb_.translation() = Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
          v_ = m->twist.twist.linear.x, have_odom_ = true;
        });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this] { tick(); });
    iter_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(int(period * 1000)), [this] { iterate(); });
  }

 private:
  bool lookupAll() {
    try {
      for (auto& [n, s] : sensors_) {
        if (s.have_tf) continue;
        s.T_nom = tf2::transformToEigen(tf_buffer_->lookupTransform("base_link", s.frame, tf2::TimePointZero));
        s.T_est = s.T_nom, s.have_tf = true;
      }
      if (!ref_ok_)
        T_bref_ = tf2::transformToEigen(tf_buffer_->lookupTransform("base_link", "lidar_" + ref_name_, tf2::TimePointZero)), ref_ok_ = true;
    } catch (const std::exception&) {
      return false;
    }
    return true;
  }

  // Range crop + self-filter (the corner LiDARs see the vehicle's own platform and container).
  std::vector<Eigen::Vector3d> downsample(const std::vector<Eigen::Vector3d>& c, const Eigen::Isometry3d& T_base_sensor,
                                          double voxel, double rmin, double rmax) const {
    std::vector<Eigen::Vector3d> p;
    p.reserve(c.size());
    for (const auto& v : c) {
      const Eigen::Vector3d b = T_base_sensor * v;
      const double r = v.norm();
      const bool self = b.x() > -rear_ - 0.8 && b.x() < front_ + 0.8 && std::abs(b.y()) < width_ / 2 + 0.8 && b.z() < height_ + 1.2;
      if (r > rmin && r < rmax && !self) p.push_back(v);
    }
    return voxelDownsample(p, voxel);
  }

  void tick() {
    if (!have_odom_ || !lookupAll()) return;
    if (state_ == "ground") {
      if (std::abs(v_) > 0.05) return;
      bool all = true;
      for (auto& [n, s] : sensors_) {
        if (s.latest.empty()) { all = false; continue; }
        if (s.ground_scans < n_ground_) {
          for (const auto& pl : s.latest) {  // ground candidates: low in the (nominal) base frame, near the sensor
            const Eigen::Vector3d pb = s.T_nom * pl;
            if (pb.z() < 0.6 && pl.norm() < 25.0 && pl.norm() > 2.0) s.ground_pts.push_back(pl);
          }
          s.latest.clear(), ++s.ground_scans;
        }
        all = all && s.ground_scans >= n_ground_;
      }
      if (all) {
        for (auto& [n, s] : sensors_) {
          const Plane g = fitGroundPlane(s.ground_pts);
          s.T_est = applyGroundPlane(s.T_nom, g);
          RCLCPP_INFO(get_logger(), "%s ground plane: %d inliers -> roll/pitch/z set", n.c_str(), g.inliers);
        }
        state_ = "collect", last_kf_ = T_mb_.translation();
      }
    } else if (state_ == "collect") {
      if (std::abs(v_) > 0.3) moved_ = true;
      const bool route_done = moved_ && std::abs(v_) < 0.02 && ++still_ticks_ > 20;
      if ((T_mb_.translation() - last_kf_).norm() >= spacing_ && n_kf_ < max_kf_) {
        for (auto& [n, s] : sensors_) {
          if (s.latest.empty()) continue;
          s.kfs.push_back({T_mb_, downsample(s.latest, s.T_est, 0.35, 2.0, 60.0)});
          s.latest.clear();
        }
        if (!ref_latest_.empty()) {
          const Eigen::Isometry3d T = T_mb_ * T_bref_;
          for (const auto& p : downsample(ref_latest_, T_bref_, 0.2, 3.0, 70.0)) ref_pts_.push_back(T * p);
          ref_latest_.clear();
        }
        last_kf_ = T_mb_.translation(), ++n_kf_;
      }
      if ((n_kf_ >= max_kf_ || route_done) && n_kf_ >= 8) {
        map_.build(voxelDownsample(ref_pts_, 0.2), 1.0, 0.05, 5);
        ref_pub_->publish(xyzMsg(voxelDownsample(ref_pts_, 0.4), "map", now()));
        RCLCPP_INFO(get_logger(), "reference map: %zu points -> %zu voxels from %d keyframes", ref_pts_.size(), map_.size(), n_kf_);
        state_ = "register";
      }
    }
    publish();
  }

  void iterate() {
    if (state_ != "register" && state_ != "refine") return;
    const bool refining = state_ == "refine";
    double max_step = 0;
    for (auto& [n, s] : sensors_) {
      const StepResult r = refining ? refineStep6(map_, s.kfs, s.T_est) : refineStep(map_, s.kfs, s.T_est);
      s.T_est = r.T, s.rms = r.rms, s.matched = r.matched, max_step = std::max(max_step, r.step);
    }
    if (++iter_ >= (refining ? max_iter_6dof_ : max_iter_) || (iter_ > 5 && max_step < 2e-5)) {
      if (refining) {
        state_ = "done";
        writeYaml();
      } else {
        state_ = "refine";
        iter_ = 0;
      }
    }
  }

  void publish() {
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    for (auto& [n, s] : sensors_) {
      auto t = tf2::eigenToTransform(s.T_est);
      t.header.stamp = now(), t.header.frame_id = "base_link", t.child_frame_id = "calib/" + s.frame;
      tfs.push_back(t);
    }
    tf_pub_->sendTransform(tfs);
    if (++pub_count_ % 3 == 0)  // accumulated keyframe clouds with the *current* extrinsic estimate
      for (auto& [n, s] : sensors_) {
        std::vector<Eigen::Vector3d> out;
        for (const auto& kf : s.kfs) {
          const Eigen::Isometry3d T = kf.T_map_base * s.T_est;
          for (const auto& p : kf.pts) out.push_back(T * p);
        }
        if (s.kfs.empty() && !s.latest.empty()) {  // before the first keyframe: the live scan
          const Eigen::Isometry3d T = T_mb_ * s.T_est;
          for (const auto& q : s.latest) out.push_back(T * q);
        }
        cloud_pubs_[n]->publish(xyzMsg(out, "map", now()));
      }
    std::ostringstream js;
    js.precision(4);
    js << std::fixed << "{\"state\":\"" << state_ << "\",\"keyframes\":" << n_kf_ << ",\"max_keyframes\":" << max_kf_
       << ",\"iteration\":" << iter_ << ",\"max_iterations\":" << ((state_ == "refine" || state_ == "done") ? max_iter_6dof_ : max_iter_)
       << ",\"lidars\":{";
    bool first = true;
    for (auto& [n, s] : sensors_) {  // estimate (base_link -> sensor) and the correction applied to the nominal one
      const auto [rc, tc] = poseError(s.T_nom, s.T_est);
      const Eigen::Vector3d t = s.T_est.translation(), rpy = rpyFromQuat(Eigen::Quaterniond(s.T_est.linear()));
      js << (first ? "" : ",") << "\"" << n << "\":{\"frame\":\"" << s.frame << "\",\"xyz\":[" << t.x() << "," << t.y() << ","
         << t.z() << "],\"rpy\":[" << rpy.x() << "," << rpy.y() << "," << rpy.z() << "],\"correction_deg\":" << rad2deg(rc)
         << ",\"correction_cm\":" << 100 * tc << ",\"rms_cm\":" << 100 * s.rms << ",\"matched\":" << s.matched << "}";
      first = false;
    }
    js << "}}";
    std_msgs::msg::String m;
    m.data = js.str();
    metrics_pub_->publish(m);
  }

  void writeYaml() {
    if (yaml_.empty()) return;
    std::ofstream f(yaml_);
    if (!f) {
      RCLCPP_WARN(get_logger(), "cannot write %s", yaml_.c_str());
      return;
    }
    f << "# base_link -> lidar extrinsics (x y z [m], roll pitch yaw [rad]) from iad_calibration\n";
    for (auto& [n, s] : sensors_) {
      const Eigen::Vector3d t = s.T_est.translation(), rpy = rpyFromQuat(Eigen::Quaterniond(s.T_est.linear()));
      f << s.frame << ": {x: " << t.x() << ", y: " << t.y() << ", z: " << t.z() << ", roll: " << rpy.x() << ", pitch: " << rpy.y()
        << ", yaw: " << rpy.z() << "}  # point-to-plane residual " << 100 * s.rms << " cm\n";
    }
    RCLCPP_INFO(get_logger(), "calibration done after %d refinement iterations -> %s", iter_, yaml_.c_str());
  }

  double front_ = 10.0, rear_ = 3.0, width_ = 3.0, height_ = 4.2;
  std::map<std::string, Sensor> sensors_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_pubs_;
  std::string ref_name_, ref_frame_, yaml_, state_ = "ground";
  double spacing_, v_ = 0;
  int max_kf_, n_ground_, max_iter_, max_iter_6dof_, n_kf_ = 0, iter_ = 0, still_ticks_ = 0;
  uint64_t pub_count_ = 0;
  bool have_odom_ = false, ref_ok_ = false, moved_ = false;
  Eigen::Isometry3d T_mb_ = Eigen::Isometry3d::Identity(), T_bref_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d last_kf_ = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> ref_latest_;
  std::vector<Eigen::Vector3d> ref_pts_;
  SurfelMap map_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_, iter_timer_;
};

}  // namespace iad::calibration

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iad::calibration::CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
