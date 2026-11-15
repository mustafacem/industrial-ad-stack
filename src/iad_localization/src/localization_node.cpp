// Localization for GNSS-degraded industrial sites: EKF (wheel odometry + gyro + chi2-gated GNSS) fused with
// multi-resolution NDT scan matching against a prior point-cloud map. Standalone: standard inputs only.
//
// In : /sensing/lidar/top/points (PointCloud2), /sensing/imu/data (Imu), /sensing/gnss/fix (NavSatFix),
//      /vehicle/status/velocity (TwistWithCovarianceStamped, wheel speed), /tf_static (base_link -> sensors),
//      map: `map_file` (PCD) or /map/pointcloud_map; optional /initialpose (PoseWithCovarianceStamped)
// Out: /localization/kinematic_state (Odometry), TF map -> base_link, /localization/pose_with_covariance,
//      /map/pointcloud_map (latched, when loaded from file), ~/ndt/aligned_points, ~/metrics (JSON),
//      ~/debug/paths/<variant>, ~/debug/markers
// Initialisation: `initial_pose` parameter, or /initialpose, or automatically from the first GNSS fix with
// an NDT yaw search (16 hypotheses, best NVTL wins).
// For comparison the node also runs three reduced filters on the same inputs: GNSS-gated (no NDT),
// GNSS-raw (no gating, no NDT) and dead reckoning. Their poses are in ~/metrics; scoring them against
// ground truth is done outside this package (iad_eval).
#include <deque>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "iad_localization/ekf.hpp"
#include "iad_localization/global_init.hpp"
#include "iad_localization/ndt.hpp"
#include "iad_localization/util.hpp"

namespace iad::localization {
using visualization_msgs::msg::Marker;

class LocalizationNode : public rclcpp::Node {
 public:
  static constexpr int kFused = 0, kGated = 1, kRaw = 2, kDr = 3, kN = 4;

  explicit LocalizationNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("localization", opt) {
    front_ = declare_parameter("vehicle.front", 10.0), rear_ = declare_parameter("vehicle.rear", 3.0);
    width_ = declare_parameter("vehicle.width", 3.0);
    use_ndt_ = declare_parameter("use_ndt", true);
    ndt_res_ = declare_parameter("ndt_resolution", 2.0);
    ndt_voxel_ = declare_parameter("scan_voxel", 1.0);
    min_intensity_ = declare_parameter("min_intensity", 12.0);
    init_window_ = declare_parameter("init_search_window", 5.0);
    init_pose_ = declare_parameter("initial_pose", std::vector<double>{});
    origin_ = GeoOrigin(declare_parameter("map_origin.latitude", 35.6206), declare_parameter("map_origin.longitude", 139.79),
                        declare_parameter("map_origin.altitude", 0.0));
    map_frame_ = declare_parameter("map_frame", "map"), base_frame_ = declare_parameter("base_frame", "base_link");
    publish_tf_ = declare_parameter("publish_tf", true);
    f_[kRaw].params().gate = false;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 20);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/pose_with_covariance", 10);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/ndt/aligned_points", rclcpp::SensorDataQoS().keep_last(2));
    metrics_pub_ = create_publisher<std_msgs::msg::String>("~/metrics", 10);
    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 5);
    for (const char* n : {"fused", "gnss_gated", "gnss_raw", "dead_reckoning"})
      path_pubs_.push_back(create_publisher<nav_msgs::msg::Path>(std::string("~/debug/paths/") + n, 2));

    const std::string map_file = declare_parameter("map_file", "");
    if (!map_file.empty()) {
      std::vector<Eigen::Vector3d> pts;
      std::vector<float> inten;
      if (!loadPcd(map_file, pts, &inten)) throw std::runtime_error("cannot read PCD map " + map_file);
      buildMap(pts);
      map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/pointcloud_map", latched);
      map_pub_->publish(cloudMsg(pts, inten, map_frame_, now()));
    } else {
      map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/map/pointcloud_map", latched,
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
            std::vector<Eigen::Vector3d> pts;
            readCloud(*m, pts);
            buildMap(pts);
          });
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(declare_parameter("imu_topic", "/sensing/imu/data"), 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(*m); });
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        declare_parameter("velocity_topic", "/vehicle/status/velocity"), 20,
        [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) { v_wheel_ = m->twist.twist.linear.x; });
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(declare_parameter("gnss_topic", "/sensing/gnss/fix"), 10,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) { onGnss(*m); });
    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
          const double yaw = rpyFromQuat(Eigen::Quaterniond(m->pose.pose.orientation.w, m->pose.pose.orientation.x,
                                                             m->pose.pose.orientation.y, m->pose.pose.orientation.z)).z();
          pending_init_ = Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y, yaw);
          RCLCPP_INFO(get_logger(), "/initialpose (%.1f, %.1f, %.0f deg)", pending_init_->x(), pending_init_->y(), rad2deg(yaw));
        });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(declare_parameter("lidar_topic", "/sensing/lidar/top/points"),
        rclcpp::SensorDataQoS().keep_last(2), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onCloud(*m); });
    if (init_pose_.size() == 3) pending_init_ = Eigen::Vector3d(init_pose_[0], init_pose_[1], init_pose_[2]);
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this] { publishViz(); });
  }

 private:
  void buildMap(const std::vector<Eigen::Vector3d>& pts) {
    Stopwatch sw;
    map_.build(pts, ndt_res_);
    field_.build(pts);
    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "NDT map: %zu points -> %zu / %zu voxel Gaussians (%.0f / %.0f m), %zu structure points "
                "for global init, in %.0f ms", pts.size(), map_.coarse.size(), map_.fine.size(), 2 * ndt_res_, ndt_res_,
                field_.structurePoints(), sw.ms());
  }

  bool lever() {
    if (lever_ok_) return true;
    try {
      lever3_ = tf2::transformToEigen(tf_buffer_->lookupTransform(base_frame_, "gnss_link", tf2::TimePointZero)).translation();
      lever_ok_ = true;
    } catch (const std::exception&) {
    }
    return lever_ok_;
  }

  void initAll(double t, double x, double y, double yaw, double std_xy, const char* how) {
    for (auto& f : f_) f.init(t, x, y, yaw, std_xy, 0.05);
    for (auto& p : paths_) p.clear();
    RCLCPP_INFO(get_logger(), "initialised (%s) at (%.1f, %.1f, %.0f deg)", how, x, y, rad2deg(yaw));
  }

  void onImu(const sensor_msgs::msg::Imu& m) {
    const double t = rclcpp::Time(m.header.stamp).seconds();
    if (pending_init_) {
      initAll(t, pending_init_->x(), pending_init_->y(), pending_init_->z(), 0.5, "given pose");
      pending_init_.reset();
    }
    if (!f_[kFused].initialized()) return;
    for (auto& f : f_) f.predict(t, v_wheel_, m.angular_velocity.z);
    // Roll/pitch from the accelerometer (gravity direction, longitudinal acceleration removed), low-passed;
    // NDT overrides them whenever it matches.
    const double dt = imu_t_ > 0 ? std::clamp(t - imu_t_, 0.0, 0.05) : 0.0;
    const double a_long = dt > 0 ? (v_wheel_ - v_prev_) / std::max(dt, 1e-3) : 0.0;
    imu_t_ = t, v_prev_ = v_wheel_, yaw_rate_ = m.angular_velocity.z - f_[kFused].x()(3);
    const double fx = m.linear_acceleration.x - std::clamp(a_long, -3.0, 3.0), fy = m.linear_acceleration.y,
                 fz = m.linear_acceleration.z;
    if (t - last_ndt_t_ > 1.0 && fz > 5.0) {
      const double k = std::min(1.0, dt / 1.5);
      pitch_ += k * (std::atan2(-fx, std::hypot(fy, fz)) - pitch_), roll_ += k * (std::atan2(fy, fz) - roll_);
    }
    if (++imu_count_ % 2 == 0) publishPose(m.header.stamp);  // 50 Hz
  }

  void onGnss(const sensor_msgs::msg::NavSatFix& m) {
    using St = sensor_msgs::msg::NavSatStatus;
    if (m.status.status < St::STATUS_FIX || !lever()) return;
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const Eigen::Vector3d enu = origin_.toEnu(m.latitude, m.longitude, m.altitude);
    gnss_status_ = m.status.status == St::STATUS_GBAS_FIX ? "RTK_FIX" : m.status.status == St::STATUS_SBAS_FIX ? "RTK_FLOAT" : "SINGLE";
    const Eigen::Vector2d z = enu.head<2>();
    last_fix_ = enu, have_fix_ = true;
    if (!f_[kFused].initialized()) return;  // auto-init happens in onCloud (needs a scan for the yaw search)
    const Eigen::Matrix2d R = Eigen::Vector2d(std::max(1e-4, m.position_covariance[0]), std::max(1e-4, m.position_covariance[4])).asDiagonal();
    // A 4-5 m mast tilts with the vehicle: the horizontal lever arm is the roll/pitch-rotated one.
    const Eigen::Vector3d lv = (Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())) * lever3_;
    for (int k : {kFused, kGated, kRaw}) f_[k].updateGnss(t, z, R, lv.head<2>());
    if (t - last_ndt_t_ > 1.0) z_ += 0.2 * ((enu.z() - lv.z()) - z_);  // height from GNSS while NDT is out
    fixes_.push_back({z.x(), z.y(), gnss_status_ == "RTK_FIX" ? 0 : gnss_status_ == "RTK_FLOAT" ? 1 : 2});
    if (fixes_.size() > 3000) fixes_.pop_front();
  }

  std::vector<Eigen::Vector3d> scanPoints(const sensor_msgs::msg::PointCloud2& m, const Eigen::Isometry3d& T_bl) const {
    std::vector<Eigen::Vector3d> raw, pts;
    std::vector<float> inten;
    readCloud(m, raw, &inten);
    pts.reserve(raw.size());
    for (size_t k = 0; k < raw.size(); ++k) {
      if (inten[k] < min_intensity_) continue;  // weak echoes (dust, spray) never belong to the map
      const double r = raw[k].norm();
      if (r < 3.0 || r > 80.0) continue;
      const Eigen::Vector3d pb = T_bl * raw[k];
      if (pb.x() > -rear_ - 1 && pb.x() < front_ + 1 && std::abs(pb.y()) < width_ / 2 + 1) continue;
      pts.push_back(pb);
    }
    return voxelDownsample(pts, ndt_voxel_);
  }

  // Global initialisation: GNSS gives the position (metres of error), a correlative search over all
  // headings finds the pose on the map's structure, NDT polishes it.
  void autoInit(double t, const std::vector<Eigen::Vector3d>& src) {
    if (!have_fix_ || !lever() || !map_ready_ || src.size() < 200) return;
    std::vector<Eigen::Vector2d> structure;
    for (const auto& p : src)
      if (p.z() > 0.8) structure.emplace_back(p.x(), p.y());
    if (structure.size() < 60) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "auto-init: only %zu structure points in view", structure.size());
      return;
    }
    if (structure.size() > 400) {  // evenly thin out for speed
      std::vector<Eigen::Vector2d> thin;
      for (size_t k = 0; k < structure.size(); k += structure.size() / 400 + 1) thin.push_back(structure[k]);
      structure.swap(thin);
    }
    Stopwatch sw;
    const auto g = field_.search(structure, last_fix_.x(), last_fix_.y(), lever3_.head<2>(), init_window_);
    const NdtResult r = map_.align(src, makeIso(g.x, g.y, last_fix_.z() - lever3_.z(), 0, 0, g.yaw));
    if (g.score < 0.25 || r.nvtl < 0.2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "auto-init: weak match (field %.2f, NVTL %.2f), retrying", g.score, r.nvtl);
      return;
    }
    const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(r.T.linear()));
    z_ = r.T.translation().z(), roll_ = rpy.x(), pitch_ = rpy.y();
    char how[128];
    std::snprintf(how, sizeof(how), "GNSS %s + correlative search %.0f ms (score %.2f) + NDT (NVTL %.2f)", gnss_status_.c_str(),
                  sw.ms(), g.score, r.nvtl);
    initAll(t, r.T.translation().x(), r.T.translation().y(), rpy.z(), 0.3, how);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2& m) {
    if (!use_ndt_ || !map_ready_) return;
    Eigen::Isometry3d T_bl;
    try {
      T_bl = tf2::transformToEigen(tf_buffer_->lookupTransform(base_frame_, m.header.frame_id, tf2::TimePointZero));
    } catch (const std::exception&) {
      return;
    }
    Stopwatch sw;
    const auto src = scanPoints(m, T_bl);
    const double t = rclcpp::Time(m.header.stamp).seconds();
    if (!f_[kFused].initialized()) {
      autoInit(t, src);
      return;
    }
    const auto& x = f_[kFused].x();
    const NdtResult r = map_.align(src, makeIso(x(0), x(1), z_, roll_, pitch_, x(2)));
    ndt_ms_ = sw.ms(), ndt_iter_ = r.iterations, ndt_nvtl_ = r.nvtl, ndt_pts_ = int(src.size());
    if (r.nvtl < 0.15 || r.matched < 100) {
      ++ndt_bad_;
      return;
    }
    const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(r.T.linear()));
    f_[kFused].updatePose(t, Eigen::Vector3d(r.T.translation().x(), r.T.translation().y(), rpy.z()), r.cov_xyyaw);
    z_ = r.T.translation().z(), roll_ = rpy.x(), pitch_ = rpy.y(), last_ndt_t_ = t;
    ndt_std_ = std::sqrt(std::max(r.cov_xyyaw(0, 0), r.cov_xyyaw(1, 1)));
    if (aligned_pub_->get_subscription_count() > 0) {
      std::vector<Eigen::Vector3d> out;
      out.reserve(src.size());
      for (const auto& p : src) out.push_back(r.T * p);
      aligned_pub_->publish(cloudMsg(out, std::vector<float>(out.size(), 100.f), map_frame_, m.header.stamp));
    }
  }

  void publishPose(const builtin_interfaces::msg::Time& st) {
    const auto& x = f_[kFused].x();
    const Eigen::Isometry3d T = makeIso(x(0), x(1), z_, roll_, pitch_, x(2));
    geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(T);
    tf.header.stamp = st, tf.header.frame_id = map_frame_, tf.child_frame_id = base_frame_;
    if (publish_tf_) tf_pub_->sendTransform(tf);
    nav_msgs::msg::Odometry o;
    o.header = tf.header, o.child_frame_id = base_frame_;
    o.pose.pose = tf2::toMsg(T);
    const auto& P = f_[kFused].P();
    o.pose.covariance[0] = P(0, 0), o.pose.covariance[1] = P(0, 1), o.pose.covariance[6] = P(1, 0);
    o.pose.covariance[7] = P(1, 1), o.pose.covariance[35] = P(2, 2);
    o.pose.covariance[14] = o.pose.covariance[21] = o.pose.covariance[28] = 0.01;
    o.twist.twist.linear.x = x(4) * v_wheel_, o.twist.twist.angular.z = yaw_rate_;
    odom_pub_->publish(o);
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header = o.header, p.pose = o.pose;
    pose_pub_->publish(p);
    if (++n_trail_ % 12 == 0) {  // ~4 Hz trails
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_, ps.pose.orientation.w = 1.0;
      for (int k = 0; k < kN; ++k) {
        ps.pose.position.x = f_[k].x()(0), ps.pose.position.y = f_[k].x()(1), ps.pose.position.z = z_ + 0.2 + 0.05 * k;
        paths_[size_t(k)].push_back(ps);
      }
    }
  }

  Marker ellipse(int id, const Ekf& f, float r, float gc, float b, double z) {
    Marker m;
    m.header.frame_id = map_frame_, m.ns = "cov", m.id = id, m.type = Marker::CYLINDER;
    const Eigen::Matrix2d P = f.P().topLeftCorner<2, 2>();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(P);
    const double a = std::atan2(es.eigenvectors()(1, 1), es.eigenvectors()(0, 1));
    m.pose.position.x = f.x()(0), m.pose.position.y = f.x()(1), m.pose.position.z = z;
    m.pose.orientation.z = std::sin(a / 2), m.pose.orientation.w = std::cos(a / 2);
    m.scale.x = std::max(0.3, 2 * 3 * std::sqrt(std::max(0.0, es.eigenvalues()(1))));  // 3 sigma
    m.scale.y = std::max(0.3, 2 * 3 * std::sqrt(std::max(0.0, es.eigenvalues()(0))));
    m.scale.z = 0.08;
    m.color.r = r, m.color.g = gc, m.color.b = b, m.color.a = 0.45f;
    return m;
  }

  void publishViz() {
    if (!f_[kFused].initialized()) return;
    const auto stamp = get_clock()->now();
    for (size_t k = 0; k < path_pubs_.size(); ++k) {
      nav_msgs::msg::Path p;
      p.header.frame_id = map_frame_, p.header.stamp = stamp;
      p.poses.assign(paths_[k].begin(), paths_[k].end());
      path_pubs_[k]->publish(p);
    }
    visualization_msgs::msg::MarkerArray ma;
    ma.markers.push_back(ellipse(0, f_[kFused], 0.2f, 1.0f, 0.5f, z_ + 0.3));
    ma.markers.push_back(ellipse(1, f_[kGated], 1.0f, 0.6f, 0.1f, z_ + 0.25));
    Marker fx;
    fx.header.frame_id = map_frame_, fx.ns = "gnss", fx.id = 0, fx.type = Marker::SPHERE_LIST;
    fx.pose.orientation.w = 1.0, fx.scale.x = fx.scale.y = fx.scale.z = 0.9;
    for (const auto& f : fixes_) {
      geometry_msgs::msg::Point p;
      p.x = f.x, p.y = f.y, p.z = z_ + 0.4;
      std_msgs::msg::ColorRGBA c;
      c.a = 0.9f;
      if (f.status == 0) c.r = 0.3f, c.g = 0.95f, c.b = 0.45f;
      else if (f.status == 1) c.r = 1.0f, c.g = 0.85f, c.b = 0.2f;
      else c.r = 1.0f, c.g = 0.3f, c.b = 0.25f;
      fx.points.push_back(p), fx.colors.push_back(c);
    }
    ma.markers.push_back(fx);
    viz_pub_->publish(ma);
    const auto& s0 = f_[kFused].stats();
    const auto& s1 = f_[kGated].stats();
    std::ostringstream js;
    js.precision(4);
    js << std::fixed << "{\"t\":" << f_[kFused].t() << ",\"gnss_status\":\"" << gnss_status_ << "\",\"ndt_ms\":" << ndt_ms_
       << ",\"ndt_iter\":" << ndt_iter_ << ",\"ndt_nvtl\":" << ndt_nvtl_ << ",\"ndt_pts\":" << ndt_pts_
       << ",\"ndt_std\":" << ndt_std_ << ",\"ndt_bad\":" << ndt_bad_ << ",\"ndt_age\":" << f_[kFused].t() - last_ndt_t_
       << ",\"gnss_rejected_fused\":" << s0.gnss_rejected << ",\"gnss_rejected_gated\":" << s1.gnss_rejected
       << ",\"ndt_used\":" << s0.ndt_used << ",\"ndt_rejected\":" << s0.ndt_rejected << ",\"gyro_bias\":" << f_[kFused].x()(3)
       << ",\"odom_scale\":" << f_[kFused].x()(4) << ",\"z\":" << z_;
    const char* names[] = {"fused", "gnss_gated", "gnss_raw", "dead_reckoning"};
    for (int k = 0; k < kN; ++k) {
      const auto& x = f_[k].x();
      js << ",\"" << names[k] << "\":{\"x\":" << x(0) << ",\"y\":" << x(1) << ",\"yaw\":" << x(2)
         << ",\"std\":" << std::sqrt(std::max(f_[k].P()(0, 0), f_[k].P()(1, 1))) << "}";
    }
    js << "}";
    std_msgs::msg::String sm;
    sm.data = js.str();
    metrics_pub_->publish(sm);
  }

  struct Fix {
    double x, y;
    int status;
  };
  double front_, rear_, width_;
  bool use_ndt_, map_ready_ = false, lever_ok_ = false, publish_tf_ = true, have_fix_ = false;
  double ndt_res_, ndt_voxel_, min_intensity_, init_window_;
  LikelihoodField field_;
  std::vector<double> init_pose_;
  std::optional<Eigen::Vector3d> pending_init_;
  GeoOrigin origin_;
  std::string map_frame_, base_frame_, gnss_status_ = "NONE";
  std::array<Ekf, kN> f_;
  NdtPyramid map_;
  std::deque<Fix> fixes_;
  std::array<std::vector<geometry_msgs::msg::PoseStamped>, kN> paths_;
  Eigen::Vector3d lever3_ = Eigen::Vector3d::Zero(), last_fix_ = Eigen::Vector3d::Zero();
  double last_ndt_t_ = -1e9, imu_t_ = -1, v_prev_ = 0, yaw_rate_ = 0;
  double v_wheel_ = 0, z_ = 0, roll_ = 0, pitch_ = 0;
  double ndt_ms_ = 0, ndt_nvtl_ = 0, ndt_std_ = 0;
  int ndt_iter_ = 0, ndt_pts_ = 0, ndt_bad_ = 0;
  size_t n_trail_ = 0, imu_count_ = 0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_, map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_, lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace iad::localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iad::localization::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
