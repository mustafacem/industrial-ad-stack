// iad_sim: sensor-level simulator node. Owns the clock (/clock), steps world + vehicle at 100 Hz in
// sim time, and publishes LiDAR / IMU / GNSS / wheel odometry, ground truth, prior maps and visuals.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <execinfo.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "iad_sim/maps.hpp"
#include "iad_sim/sensors.hpp"
#include "iad_sim/vehicle_sim.hpp"
#include "iad_sim/viz.hpp"

namespace iad::sim {
using namespace std::chrono_literals;

constexpr double kDt = 0.01;   // physics step [s]
constexpr double kT0 = 100.0;  // sim clock starts here (avoids time 0 == "latest" in TF lookups)

geometry_msgs::msg::TransformStamped toTf(const Eigen::Isometry3d& T, const std::string& parent,
                                          const std::string& child, const builtin_interfaces::msg::Time& st) {
  geometry_msgs::msg::TransformStamped m;
  m.header.stamp = st, m.header.frame_id = parent, m.child_frame_id = child;
  const Eigen::Quaterniond q(T.linear());
  m.transform.translation.x = T.translation().x(), m.transform.translation.y = T.translation().y();
  m.transform.translation.z = T.translation().z();
  m.transform.rotation.x = q.x(), m.transform.rotation.y = q.y(), m.transform.rotation.z = q.z(), m.transform.rotation.w = q.w();
  return m;
}

class SimNode : public rclcpp::Node {
 public:
  SimNode() : Node("iad_sim") {
    scenario_ = declare_parameter("scenario", "quarry");
    seed_ = declare_parameter("seed", 7);
    vehicle_name_ = declare_parameter("vehicle", scenario_ == "port" ? "container_agv" : "haul_truck");
    start_pose_ = declare_parameter("start_pose", "start");
    command_source_ = declare_parameter("command_source", "autopilot");
    route_name_ = declare_parameter("autopilot_route", scenario_ == "port" ? "loc_loop" : "haul");
    ap_speed_ = declare_parameter("autopilot_speed", 0.0);
    ap_loop_ = declare_parameter("autopilot_loop", false);
    ap_delay_ = declare_parameter("autopilot_delay", 0.0);  // parked this long first (e.g. ground-plane calibration)
    publish_base_tf_ = declare_parameter("publish_base_link_tf", true);
    corner_lidars_ = declare_parameter("corner_lidars", false);
    extr_err_deg_ = declare_parameter("extrinsic_error_deg", 0.0);
    extr_err_m_ = declare_parameter("extrinsic_error_m", 0.0);
    ego_dust_ = declare_parameter("ego_dust", true);
    time_scale_ = declare_parameter("time_scale", 1.0);
    paused_ = declare_parameter("start_paused", false);
    auto_start_ = declare_parameter("auto_start_after", -1.0);
    duration_ = declare_parameter("duration", -1.0);
    const bool loaded = declare_parameter("loaded", false);
    const double delay = declare_parameter("actuator_delay", -1.0);
    tdisp_ = declare_parameter("terrain_display_res", scenario_ == "port" ? 4.0 : 2.5);
    lidar_hres_ = declare_parameter("lidar_hres", 0.3);
    declare_parameter("ego_ghost", false);
    lidar_rate_div_ = int(std::lround(1.0 / (kDt * declare_parameter("lidar_rate", 10.0))));

    Stopwatch sw;
    world_ = makeWorld(scenario_, uint32_t(seed_));
    // Demo variants: unmapped obstacles that only some demos should meet (the site map does not know them).
    if (declare_parameter("planning_obstacles", false)) {  // a container set down mid-aisle (port planning demo)
      Box b;
      b.c = Eigen::Vector3d(143.0, 23.5, 1.3), b.half = Eigen::Vector3d(6.1, 1.22, 1.3), b.yaw = 0.0;
      b.color = {0.86f, 0.46f, 0.12f, 1.f}, b.refl = 0.5f, b.in_map = false;
      world_.boxes.push_back(b);
    }
    vp_ = vehicle_name_ == "container_agv" ? containerAgvParams() : haulTruckParams();
    if (delay >= 0) vp_.delay = delay;
    veh_ = std::make_unique<VehicleSim>(vp_, kDt);
    veh_->setLoaded(loaded);
    const NamedPose* sp = world_.pose(start_pose_);
    if (!sp) sp = world_.pose("start");
    veh_->reset(Pose2{sp->x, sp->y, sp->yaw}, world_.terrain);
    route_ = world_.route(route_name_);
    setupSensors();
    ego_boxes_ = egoBodyBoxes(vp_);
    rng_ = FastRng(uint64_t(seed_) * 7919u + 1u);
    mt_.seed(uint32_t(seed_) + 5u);

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/data", 50);
    gnss_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 10);
    gnss_status_pub_ = create_publisher<std_msgs::msg::String>("/sensing/gnss/status", 10);
    status_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/vehicle/status", 20);
    gt_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/ground_truth/odom", 20);
    debug_pub_ = create_publisher<std_msgs::msg::String>("/sim/debug", 10);
    dust_pub_ = create_publisher<visualization_msgs::msg::Marker>("/sim/dust", 2);
    for (const auto& l : lidars_)
      lidar_pubs_.push_back(create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/" + l.name + "/points", rclcpp::SensorDataQoS().keep_last(2)));
    world_pub_ = create_publisher<MarkerArray>("/sim/world/markers", rclcpp::QoS(64).transient_local().reliable());
    ego_pub_ = create_publisher<MarkerArray>("/sim/ego/markers", latched);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/pointcloud_map", latched);
    map_viz_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/pointcloud_map_viz", latched);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map/costmap", latched);
    route_pub_ = create_publisher<nav_msgs::msg::Path>("/sim/route", latched);
    poses_pub_ = create_publisher<std_msgs::msg::String>("/sim/poses", latched);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/command", 10, [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
          ext_cmd_.steer = m->drive.steering_angle, ext_cmd_.accel = m->drive.acceleration, ext_cmd_t_ = t_;
          ext_cmd_.gear = m->drive.speed > 0.01 ? 1 : m->drive.speed < -0.01 ? -1 : 0;  // gear from the commanded speed sign
          have_cmd_ = true;
        });
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "/sim/start", [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
          paused_ = false, r->success = true, last_wall_ = std::chrono::steady_clock::now();
          RCLCPP_INFO(get_logger(), "sim started");
        });
    loaded_srv_ = create_service<std_srvs::srv::SetBool>(
        "/sim/set_loaded", [this](std_srvs::srv::SetBool::Request::SharedPtr q, std_srvs::srv::SetBool::Response::SharedPtr r) {
          veh_->setLoaded(q->data), r->success = true;
        });
    publishStatic();
    RCLCPP_INFO(get_logger(), "world '%s' ready in %.0f ms: %zu boxes, %zu cylinders, %zu actors, vehicle %s, %zu lidars",
                scenario_.c_str(), sw.ms(), world_.boxes.size(), world_.cylinders.size(), world_.actors.size(),
                vp_.name.c_str(), lidars_.size());
    wall_start_ = last_wall_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(4ms, [this] { tick(); });
  }

 private:
  void setupSensors() {
    auto top = [&](double x, double y, double z) {
      LidarConfig l;
      l.name = "top", l.frame = "lidar_top", l.channels = 32, l.vmin = -25, l.vmax = 15, l.hres = lidar_hres_;
      l.T_base_true = makeIso(x, y, z, 0, 0, 0);
      return l;
    };
    if (vp_.name == "container_agv") {
      lidars_.push_back(top(9.85, 0.0, 5.05));
      gnss_lever_ = Eigen::Vector3d(9.85, 0.45, 5.3);
      if (corner_lidars_) {
        struct C { const char* n; double x, y, yaw; };
        for (const C& c : {C{"front_left", 10.05, 1.55, 45}, C{"front_right", 10.05, -1.55, -45},
                           C{"rear_left", -3.05, 1.55, 135}, C{"rear_right", -3.05, -1.55, -135}}) {
          LidarConfig l;
          l.name = c.n, l.frame = std::string("lidar_") + c.n, l.channels = 16, l.vmin = -15, l.vmax = 15;
          l.hres = 0.4, l.dual = false, l.range = 80;
          l.T_base_true = makeIso(c.x, c.y, 1.95, 0.0, deg2rad(10), deg2rad(c.yaw));
          lidars_.push_back(l);
        }
      }
    } else {
      lidars_.push_back(top(5.2, 0.0, 4.15));
      gnss_lever_ = Eigen::Vector3d(4.6, -1.6, 4.1);
    }
    std::mt19937 rng(uint32_t(seed_) + 99u);
    std::uniform_real_distribution<double> U(0.5, 1.0);
    std::bernoulli_distribution B(0.5);
    for (auto& l : lidars_) {
      l.T_base_nominal = l.T_base_true;
      if (l.name == "top" || (extr_err_deg_ <= 0 && extr_err_m_ <= 0)) continue;
      Eigen::Matrix<double, 6, 1> xi;
      for (int k = 0; k < 6; ++k) xi[k] = (B(rng) ? 1 : -1) * U(rng) * (k < 3 ? deg2rad(extr_err_deg_) : extr_err_m_);
      l.T_base_nominal = l.T_base_true * expSE3(xi);
    }
  }

  // Integer nanoseconds from the step counter: monotonic, no float rounding into the next second.
  builtin_interfaces::msg::Time stamp() const {
    const int64_t ns = int64_t(kT0) * 1000000000LL + int64_t(k_) * int64_t(kDt * 1e9 + 0.5);
    builtin_interfaces::msg::Time s;
    s.sec = int32_t(ns / 1000000000LL);
    s.nanosec = uint32_t(ns % 1000000000LL);
    return s;
  }

  void publishStatic() {
    const auto st = stamp();
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    for (const auto& l : lidars_) {
      tfs.push_back(toTf(l.T_base_nominal, "base_link", l.frame, st));
      tfs.push_back(toTf(l.T_base_true, "gt/base_link", "gt/" + l.frame, st));
    }
    tfs.push_back(toTf(makeIso(gnss_lever_.x(), gnss_lever_.y(), gnss_lever_.z(), 0, 0, 0), "base_link", "gnss_link", st));
    tfs.push_back(toTf(makeIso(1.5, 0, 1.2, 0, 0, 0), "base_link", "imu_link", st));
    static_tf_->sendTransform(tfs);

    // World visuals in several messages (the topic keeps 64) so no single DDS sample is huge.
    auto terrain = terrainMarkers(world_.terrain, std::min(1.0, tdisp_), tdisp_, 50.0, "map");
    auto objects = objectMarkers(world_, "map");
    auto actors = actorMarkers(world_);
    std::vector<Marker> all;
    all.insert(all.end(), terrain.begin(), terrain.end());
    all.insert(all.end(), objects.begin(), objects.end());
    all.insert(all.end(), actors.begin(), actors.end());
    for (size_t k = 0; k < all.size(); k += 8) {
      MarkerArray ma;
      for (size_t i = k; i < std::min(all.size(), k + 8); ++i) all[i].header.stamp = st, ma.markers.push_back(all[i]);
      world_pub_->publish(ma);
    }
    std::vector<Eigen::Isometry3d> mounts;
    for (const auto& l : lidars_) mounts.push_back(l.T_base_true);
    MarkerArray ego;
    for (auto m : egoMarkers(vp_, mounts, "gt/base_link", "ego", 1.0f)) ego.markers.push_back(m);
    if (get_parameter("ego_ghost").as_bool())  // translucent copy at the *estimated* pose (localization demo)
      for (auto m : egoMarkers(vp_, mounts, "base_link", "ego_estimate", 0.35f)) ego.markers.push_back(m);
    ego_pub_->publish(ego);

    // Terrain spacing 0.5 m in the quarry: bare ground must fill the 2 m NDT voxels (>= 6 points each).
    const Cloud map = buildPointcloudMap(world_, scenario_ == "port" ? 1.0 : 0.5, 0.6, 0.5);
    map_pub_->publish(toMsg(map, "map", st));
    Cloud viz;
    for (size_t i = 0; i < map.size(); i += 3) viz.push_back(map[i]);
    map_viz_pub_->publish(toMsg(viz, "map", st));
    const CostGrid cg = buildCostmap(world_, 0.5, 5.5);
    nav_msgs::msg::OccupancyGrid og;
    og.header.frame_id = "map", og.header.stamp = st;
    og.info.resolution = float(cg.res), og.info.width = uint32_t(cg.w), og.info.height = uint32_t(cg.h);
    og.info.origin.position.x = cg.ox, og.info.origin.position.y = cg.oy, og.info.origin.orientation.w = 1.0;
    og.data.assign(cg.data.begin(), cg.data.end());
    costmap_pub_->publish(og);
    {  // named site poses (load point, dump bay, crane lanes...) for the mission manager
      std::ostringstream js;
      js << "{";
      for (size_t i = 0; i < world_.poses.size(); ++i)
        js << (i ? "," : "") << "\"" << world_.poses[i].name << "\":[" << world_.poses[i].x << "," << world_.poses[i].y << ","
           << world_.poses[i].yaw << "]";
      js << "}";
      std_msgs::msg::String ps;
      ps.data = js.str();
      poses_pub_->publish(ps);
    }
    if (route_) {  // the selected route as a path (route-following planner mode uses it as the reference)
      nav_msgs::msg::Path rp;
      rp.header.frame_id = "map", rp.header.stamp = st;
      for (size_t i = 0; i < route_->pts.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = rp.header;
        ps.pose.position.x = route_->pts[i].x(), ps.pose.position.y = route_->pts[i].y();
        ps.pose.position.z = world_.terrain.height(ps.pose.position.x, ps.pose.position.y);
        const size_t j = std::min(i + 1, route_->pts.size() - 1), k = j - 1;
        const double yaw = std::atan2(route_->pts[j].y() - route_->pts[k].y(), route_->pts[j].x() - route_->pts[k].x());
        ps.pose.orientation.z = std::sin(yaw / 2), ps.pose.orientation.w = std::cos(yaw / 2);
        rp.poses.push_back(ps);
      }
      route_pub_->publish(rp);
    }
    RCLCPP_INFO(get_logger(), "published %zu world markers, map %zu pts, costmap %dx%d", all.size(), map.size(), cg.w, cg.h);
  }

  void tick() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_wall_).count();
    last_wall_ = now;
    if (paused_ && auto_start_ > 0 && std::chrono::duration<double>(now - wall_start_).count() > auto_start_) {
      paused_ = false, auto_start_ = -1;
      RCLCPP_INFO(get_logger(), "sim auto-started");
    }
    if (paused_) {
      if (++idle_ticks_ % 25 == 0) publishIdle();
      return;
    }
    budget_ += std::min(elapsed, 0.25) * time_scale_;
    while (budget_ >= kDt) budget_ -= kDt, stepOnce();
  }

  void publishIdle() {
    const auto st = stamp();
    rosgraph_msgs::msg::Clock c;
    c.clock = st;
    clock_pub_->publish(c);
    publishPoses(st);
  }

  ActuatorCmd command() {
    if (command_source_ == "external") {
      if (!have_cmd_ || t_ - ext_cmd_t_ > 0.5) return ActuatorCmd{veh_->steer, -2.0, 0};  // watchdog: brake
      return ext_cmd_;
    }
    return autopilot();
  }

  // Scripted route follower on ground truth (drives the vehicle for perception/localization/calibration demos).
  ActuatorCmd autopilot() {
    ActuatorCmd c{veh_->steer, -1.5, 0};
    if (!route_ || route_->pts.size() < 3 || ap_done_ || t_ < ap_delay_) return c;
    c.gear = 1;
    const auto& P = route_->pts;
    const Pose2& p = veh_->pose;
    size_t best = ap_idx_;
    double bd = 1e18;
    for (size_t i = ap_idx_; i < std::min(P.size(), ap_idx_ + 60); ++i) {
      const double d = std::hypot(P[i].x() - p.x, P[i].y() - p.y);
      if (d < bd) bd = d, best = i;
    }
    ap_idx_ = best;
    const double v = veh_->v_ground, ld = std::max(8.0, 1.8 * std::abs(v));
    size_t tgt = ap_idx_;
    while (tgt + 1 < P.size() && std::hypot(P[tgt].x() - p.x, P[tgt].y() - p.y) < ld) ++tgt;
    if (ap_loop_ && P.size() - ap_idx_ < 30) ap_idx_ = 0;
    const double dx = P[tgt].x() - p.x, dy = P[tgt].y() - p.y;
    const double alpha = wrapAngle(std::atan2(dy, dx) - p.yaw), L = std::hypot(dx, dy);
    c.steer = std::atan2(2.0 * vp_.wheelbase * std::sin(alpha), std::max(L, 1.0));
    // speed: route speed, curvature limit, stop at the end
    double kmax = 0;  // curvature over a 2-point (~2 m) baseline on each side: robust to resampling jitter
    for (size_t i = ap_idx_ + 2; i + 2 < std::min(P.size(), ap_idx_ + 40); ++i) {
      const double a1 = std::atan2(P[i].y() - P[i - 2].y(), P[i].x() - P[i - 2].x());
      const double a2 = std::atan2(P[i + 2].y() - P[i].y(), P[i + 2].x() - P[i].x());
      kmax = std::max(kmax, std::abs(wrapAngle(a2 - a1)) / std::max(1.0, 0.5 * ((P[i] - P[i - 2]).norm() + (P[i + 2] - P[i]).norm())));
    }
    const double vmax = ap_speed_ > 0 ? ap_speed_ : route_->speed;
    const double remaining = double(P.size() - 1 - ap_idx_) * 1.0;
    double vt = std::min({vmax, std::sqrt(1.0 / std::max(kmax, 1e-3)), std::sqrt(2.0 * 0.5 * std::max(0.0, remaining - 3.0))});
    if (!ap_loop_ && remaining < 3.0) vt = 0, ap_done_ = std::abs(v) < 0.05;
    c.accel = std::clamp(0.7 * (vt - v) - kGravity * std::sin(veh_->pitch), -2.0, 1.8);  // + grade feed-forward
    return c;
  }

  void stepOnce() {
    const ActuatorCmd cmd = command();
    veh_->step(cmd, world_, rng_);
    world_.step(kDt, mt_);
    ++k_, t_ = double(k_) * kDt;
    egoDust();
    const auto st = stamp();
    rosgraph_msgs::msg::Clock clk;
    clk.clock = st;
    clock_pub_->publish(clk);
    publishImu(st);
    if (k_ % 2 == 0) publishPoses(st), publishStatus(st);
    if (k_ % 10 == 0) publishGnss(st), publishDust(st);
    if (k_ % lidar_rate_div_ == 0) publishLidars(st), publishDebug(st);
    if (duration_ > 0 && t_ >= duration_) {
      RCLCPP_INFO(get_logger(), "duration reached (%.1f s sim time), shutting down", t_);
      rclcpp::shutdown();
    }
  }

  void egoDust() {
    if (!ego_dust_ || std::abs(veh_->v_ground) < 2.5) return;
    const uint8_t m = world_.terrain.material(veh_->pose.x, veh_->pose.y);
    if (m != kDirt && m != kGravel && m != kMud) return;
    ego_dust_timer_ += kDt;
    if (ego_dust_timer_ < 0.25) return;
    ego_dust_timer_ = 0;
    const double c = std::cos(veh_->pose.yaw), s = std::sin(veh_->pose.yaw), back = vp_.rear + 1.5;
    const double side = (rng_.uniform() - 0.5) * vp_.width;
    const Eigen::Vector3d p(veh_->pose.x - back * c - side * s, veh_->pose.y - back * s + side * c, veh_->z + 1.3);
    world_.emitDust(p, 1.8, 0.28, 9.0, Eigen::Vector3d(world_.wind.x() - 0.1 * veh_->v_ground * c, world_.wind.y() - 0.1 * veh_->v_ground * s, 0.12));
  }

  void publishImu(const builtin_interfaces::msg::Time& st) {
    Eigen::Vector3d w, f;
    imu_.sample(veh_->omega_body, veh_->specific_force, rng_, w, f);
    sensor_msgs::msg::Imu m;
    m.header.stamp = st, m.header.frame_id = "imu_link";
    m.orientation_covariance[0] = -1;
    m.angular_velocity.x = w.x(), m.angular_velocity.y = w.y(), m.angular_velocity.z = w.z();
    m.linear_acceleration.x = f.x(), m.linear_acceleration.y = f.y(), m.linear_acceleration.z = f.z();
    for (int i = 0; i < 3; ++i) m.angular_velocity_covariance[size_t(i * 4)] = sq(imu_.gyro_noise), m.linear_acceleration_covariance[size_t(i * 4)] = sq(imu_.accel_noise);
    imu_pub_->publish(m);
  }

  void publishPoses(const builtin_interfaces::msg::Time& st) {
    const Eigen::Isometry3d T = veh_->T_map_base();
    std::vector<geometry_msgs::msg::TransformStamped> tfs{toTf(T, "map", "gt/base_link", st)};
    if (publish_base_tf_) tfs.push_back(toTf(T, "map", "base_link", st));
    for (size_t i = 0; i < world_.actors.size(); ++i) {
      const auto& a = world_.actors[i];
      tfs.push_back(toTf(makeIso(a.pose.x, a.pose.y, a.z, 0, 0, a.pose.yaw), "map", "actor_" + std::to_string(i), st));
    }
    tf_->sendTransform(tfs);
    nav_msgs::msg::Odometry o;
    o.header.stamp = st, o.header.frame_id = "map", o.child_frame_id = "gt/base_link";
    o.pose.pose.position.x = T.translation().x(), o.pose.pose.position.y = T.translation().y();
    o.pose.pose.position.z = T.translation().z();
    o.pose.pose.orientation.x = veh_->q.x(), o.pose.pose.orientation.y = veh_->q.y();
    o.pose.pose.orientation.z = veh_->q.z(), o.pose.pose.orientation.w = veh_->q.w();
    o.twist.twist.linear.x = veh_->v_ground, o.twist.twist.angular.z = veh_->yaw_rate;
    gt_pub_->publish(o);
  }

  void publishStatus(const builtin_interfaces::msg::Time& st) {
    ackermann_msgs::msg::AckermannDriveStamped m;
    m.header.stamp = st, m.header.frame_id = "base_link";
    m.drive.speed = float(veh_->v * 1.012 + 0.02 * rng_.normal());  // tyre-radius error + noise; slip not seen
    m.drive.steering_angle = float(veh_->steer + 0.003 * rng_.normal());
    m.drive.acceleration = float(veh_->accel_applied);
    status_pub_->publish(m);
  }

  void publishGnss(const builtin_interfaces::msg::Time& st) {
    const Eigen::Vector3d ant = veh_->T_map_base() * gnss_lever_;
    const auto dyn = world_.actorBoxes();
    gnss_last_ = gnss_.sample(world_, dyn, ant, 0.1, rng_);
    std::ostringstream js;
    js << "{\"status\":\"" << gnssStatusName(gnss_last_.status) << "\",\"blocked\":" << gnss_last_.sky_blocked
       << ",\"err\":" << gnss_last_.error << ",\"valid\":" << (gnss_last_.valid ? "true" : "false") << "}";
    std_msgs::msg::String s;
    s.data = js.str();
    gnss_status_pub_->publish(s);
    if (!gnss_last_.valid) return;
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = st, m.header.frame_id = "map";
    m.pose.pose.position.x = gnss_last_.pos.x(), m.pose.pose.position.y = gnss_last_.pos.y();
    m.pose.pose.position.z = gnss_last_.pos.z(), m.pose.pose.orientation.w = 1.0;
    const double s2 = sq(gnss_last_.sigma_reported);
    m.pose.covariance[0] = s2, m.pose.covariance[7] = s2, m.pose.covariance[14] = 4 * s2;
    m.pose.covariance[21] = m.pose.covariance[28] = m.pose.covariance[35] = 1e4;
    gnss_pub_->publish(m);
  }

  void publishLidars(const builtin_interfaces::msg::Time& st) {
    const Eigen::Isometry3d T = veh_->T_map_base();
    std::vector<Box> dyn = world_.actorBoxes();
    const double c = std::cos(veh_->pose.yaw), s = std::sin(veh_->pose.yaw);
    for (Box b : ego_boxes_) {
      const Eigen::Vector3d l = b.c;
      b.c = Eigen::Vector3d(veh_->pose.x + c * l.x() - s * l.y(), veh_->pose.y + s * l.x() + c * l.y(), veh_->z + l.z());
      b.yaw = veh_->pose.yaw;
      dyn.push_back(b);
    }
    lidar_ms_ = 0, lidar_pts_ = 0, lidar_dust_ = 0;
    for (size_t i = 0; i < lidars_.size(); ++i) {
      ScanStats ss;
      const Cloud cl = scanLidar(world_, dyn, T * lidars_[i].T_base_true, lidars_[i], uint64_t(seed_) * 100000ull + k_ + i, &ss);
      lidar_pubs_[i]->publish(toMsg(cl, lidars_[i].frame, st));
      lidar_ms_ += ss.ms, lidar_pts_ += ss.points, lidar_dust_ += ss.dust;
    }
  }

  void publishDust(const builtin_interfaces::msg::Time& st) {
    Marker m = dustMarker(world_, "map", rng_);
    m.header.stamp = st;
    dust_pub_->publish(m);
  }

  void publishDebug(const builtin_interfaces::msg::Time&) {
    static const char* kMat[] = {"asphalt", "gravel", "dirt", "mud", "rock", "concrete", "water"};
    std::ostringstream js;
    js.precision(4);
    js << std::fixed << "{\"t\":" << t_ << ",\"x\":" << veh_->pose.x << ",\"y\":" << veh_->pose.y << ",\"yaw\":" << veh_->pose.yaw
       << ",\"v\":" << veh_->v_ground << ",\"v_wheel\":" << veh_->v << ",\"steer\":" << veh_->steer
       << ",\"acc\":" << veh_->accel_applied << ",\"pitch\":" << veh_->pitch << ",\"roll\":" << veh_->roll
       << ",\"slip\":" << veh_->slip << ",\"mass\":" << veh_->mass()
       << ",\"mat\":\"" << kMat[std::min<int>(6, world_.terrain.material(veh_->pose.x, veh_->pose.y))]
       << "\",\"gnss\":\"" << gnssStatusName(gnss_last_.status) << "\",\"lidar_ms\":" << lidar_ms_
       << ",\"points\":" << lidar_pts_ << ",\"dust_pts\":" << lidar_dust_ << ",\"puffs\":" << world_.dust.size() << "}";
    std_msgs::msg::String s;
    s.data = js.str();
    debug_pub_->publish(s);
  }

  // parameters
  std::string scenario_, vehicle_name_, start_pose_, command_source_, route_name_;
  int64_t seed_;
  double ap_speed_, ap_delay_ = 0, extr_err_deg_, extr_err_m_, time_scale_, auto_start_, duration_, lidar_hres_, tdisp_ = 1.5;
  bool ap_loop_, publish_base_tf_, corner_lidars_, ego_dust_, paused_;
  int lidar_rate_div_ = 10;
  // state
  World world_;
  VehicleParams vp_;
  std::unique_ptr<VehicleSim> veh_;
  std::vector<LidarConfig> lidars_;
  std::vector<Box> ego_boxes_;
  Eigen::Vector3d gnss_lever_ = Eigen::Vector3d::Zero();
  GnssModel gnss_;
  GnssSample gnss_last_;
  ImuModel imu_;
  FastRng rng_{1};
  std::mt19937 mt_;
  const Route* route_ = nullptr;
  size_t ap_idx_ = 0;
  bool ap_done_ = false, have_cmd_ = false;
  ActuatorCmd ext_cmd_;
  double ext_cmd_t_ = -1, t_ = 0, budget_ = 0, ego_dust_timer_ = 0, lidar_ms_ = 0;
  size_t lidar_pts_ = 0, lidar_dust_ = 0;
  uint64_t k_ = 0, idle_ticks_ = 0;
  std::chrono::steady_clock::time_point wall_start_, last_wall_;
  // ROS
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gnss_status_pub_, debug_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dust_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs_;
  rclcpp::Publisher<MarkerArray>::SharedPtr world_pub_, ego_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, map_viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr poses_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr loaded_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace iad::sim

// Print a symbolised backtrace on a crash (glibc backtrace + -rdynamic), then re-raise.
static void onCrash(int sig) {
  void* frames[64];
  const int n = backtrace(frames, 64);
  std::fprintf(stderr, "\n*** iad_sim caught signal %d, backtrace:\n", sig);
  backtrace_symbols_fd(frames, n, 2);
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

int main(int argc, char** argv) {
  std::signal(SIGSEGV, onCrash);
  std::signal(SIGABRT, onCrash);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iad::sim::SimNode>());
  rclcpp::shutdown();
  return 0;
}
