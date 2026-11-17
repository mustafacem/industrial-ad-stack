// planner_node: site costmap (+ live perception occupancy) -> Hybrid A* + Reeds-Shepp -> smoothed,
// time-parameterised trajectory. While driving it checks the path ahead: an obstacle inside the
// horizon first gets a stop line (a crossing vehicle usually clears); if it stays, the planner replans.
// A behaviour layer caps the speed from tracked objects and dust (behaviour.hpp): follow, passing, yield, dust.
//
// In : costmap_file (nav2 map YAML/PGM) [+ elevation_file PCD], /perception/occupancy_grid,
//      /localization/kinematic_state, /planning/goal (PoseStamped) or /planning/route (nav_msgs/Path),
//      /perception/objects (autoware_perception_msgs/DetectedObjects), /perception/points/dust (PointCloud2)
// Out: /planning/trajectory (autoware_planning_msgs/Trajectory, v < 0 = reverse), /planning/path,
//      /planning/markers, /planning/search, /planning/metrics (JSON), /map/occupancy_grid (latched)
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <fstream>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "iad_planning/behaviour.hpp"
#include "iad_planning/hybrid_astar.hpp"
#include "iad_planning/velocity.hpp"

namespace iad::planning {
using visualization_msgs::msg::Marker;

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("planner", opt) {
    vp_.wheelbase = declare_parameter("vehicle.wheelbase", 4.3), vp_.width = declare_parameter("vehicle.width", 4.2);
    vp_.front = declare_parameter("vehicle.front", 6.6), vp_.rear = declare_parameter("vehicle.rear", 2.8);
    vp_.max_steer = declare_parameter("vehicle.max_steer", 0.6), vp_.max_steer_rate = declare_parameter("vehicle.max_steer_rate", 0.35);
    use_live_ = declare_parameter("use_live_occupancy", true);
    sp_.v_fwd = declare_parameter("v_forward", 6.0);
    sp_.v_rev = declare_parameter("v_reverse", 1.6);
    pp_.max_expansions = int(declare_parameter("max_expansions", 200000));
    pp_.steer_headroom = declare_parameter("steer_headroom", 0.8);
    sp_.steer_rate = vp_.max_steer_rate, sp_.wheelbase = vp_.wheelbase;
    wait_s_ = declare_parameter("wait_before_replan", 2.5);
    live_radius_ = declare_parameter("live_occupancy_radius", 40.0);
    declare_parameter("debug_dump_file", "");
    viz_scale_ = declare_parameter("viz_scale", 1.0);  // marker line widths (zoomed-out overview views)
    planner_ = std::make_unique<HybridAStar>(vp_, pp_);
    PlannerParams tight = pp_;
    tight.margin *= 0.5;
    tight_planner_ = std::make_unique<HybridAStar>(vp_, tight);
    route_wait_s_ = declare_parameter("route_wait_before_replan", 25.0);
    behave_ = declare_parameter("behaviour.enabled", true);
    bp_.pass_v = declare_parameter("behaviour.pass_speed", bp_.pass_v);
    bp_.follow_time = declare_parameter("behaviour.follow_time_gap", bp_.follow_time);
    bp_.follow_min = declare_parameter("behaviour.follow_min_gap", bp_.follow_min);
    bp_.dust_v_light = declare_parameter("behaviour.dust_speed", bp_.dust_v_light);
    bp_.dust_v_heavy = declare_parameter("behaviour.heavy_dust_speed", bp_.dust_v_heavy);
    bp_.dust_light = int(declare_parameter("behaviour.dust_echoes", bp_.dust_light));
    bp_.dust_heavy = int(declare_parameter("behaviour.heavy_dust_echoes", bp_.dust_heavy));
    objects_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
        declare_parameter("objects_topic", "/perception/objects"), 5,
        [this](autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr m) { onObjects(*m); });
    dust_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter("dust_topic", "/perception/points/dust"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onDust(*m); });

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    traj_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/planning/trajectory", latched);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/path", latched);
    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/markers", latched);
    metrics_pub_ = create_publisher<std_msgs::msg::String>("/planning/metrics", 10);
    std::string err;
    const std::string costmap = declare_parameter("costmap_file", "");
    if (!costmap.empty()) {
      if (!loadCostmap(costmap, site_, &err)) throw std::runtime_error(err);
      map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map/occupancy_grid", latched);
      nav_msgs::msg::OccupancyGrid og;
      og.header.frame_id = "map", og.header.stamp = now();
      og.info.resolution = float(site_.res), og.info.width = uint32_t(site_.w), og.info.height = uint32_t(site_.h);
      og.info.origin.position.x = site_.ox, og.info.origin.position.y = site_.oy, og.info.origin.orientation.w = 1.0;
      og.data.assign(site_.data.begin(), site_.data.end());
      map_pub_->publish(og);
      RCLCPP_INFO(get_logger(), "site costmap %dx%d @ %.2f m from %s", site_.w, site_.h, site_.res, costmap.c_str());
    } else {
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map/occupancy_grid", latched,
          [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { site_ = toGrid(*m); });
    }
    const std::string elev = declare_parameter("elevation_file", "");
    if (!elev.empty() && !loadElevation(elev, elev_)) RCLCPP_WARN(get_logger(), "no elevation from %s", elev.c_str());
    live_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(declare_parameter("occupancy_topic", "/perception/occupancy_grid"), 2,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { live_ = toGrid(*m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", "/localization/kinematic_state"), 20,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& q = m->pose.pose.orientation;
          pose_ = {m->pose.pose.position.x, m->pose.pose.position.y, yawFromQuat(Eigen::Quaterniond(q.w, q.x, q.y, q.z))};
          v_ = m->twist.twist.linear.x, have_pose_ = true;
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/planning/goal", 5, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
      const auto& q = m->pose.orientation;
      goal_ = {m->pose.position.x, m->pose.position.y, yawFromQuat(Eigen::Quaterniond(q.w, q.x, q.y, q.z))};
      state_ = "planning", have_goal_ = true, waiting_ = false;  // replans_ counts over the whole mission
    });
    // Route mode: follow a surveyed centreline (e.g. a haul road) instead of searching.
    route_sub_ = create_subscription<nav_msgs::msg::Path>("/planning/route", latched, [this](nav_msgs::msg::Path::ConstSharedPtr m) {
        Path p;
        for (const auto& ps : m->poses) {
          const auto& q = ps.pose.orientation;
          p.push_back({ps.pose.position.x, ps.pose.position.y, yawFromQuat(Eigen::Quaterniond(q.w, q.x, q.y, q.z)), 1});
        }
        if (p.size() < 2 || site_.empty()) return;
        goal_ = {p.back().x, p.back().y, p.back().yaw}, have_goal_ = true;
        full_ = timeParameterize(p, site_, sp_, 0.0), prog_ = 0, state_ = "driving", stats_ = PlanStats{};
        stats_.status = "route";
        publishTrajectory(full_, false), uncapped();  // a new reference: caps are re-applied if any
        RCLCPP_INFO(get_logger(), "route mode: %.0f m reference from /planning/route", full_.back().s);
      });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(200), [this] { tick(); });
  }

 private:
  static CostGrid toGrid(const nav_msgs::msg::OccupancyGrid& m) {
    CostGrid g(int(m.info.width), int(m.info.height), m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, 0);
    g.data.assign(m.data.begin(), m.data.end());
    return g;
  }

  double z(double x, double y) const { return elev_.empty() ? 0.0 : elev_.valueAt(x, y, 0.f); }

  CostGrid fused() const {
    CostGrid g = site_;
    if (!use_live_ || live_.empty()) return g;
    for (int j = 0; j < live_.h; ++j)
      for (int i = 0; i < live_.w; ++i) {
        if (live_.at(i, j) < kLethal) continue;
        int si, sj;
        // Near field only: far-away live returns are sparse (and moving traffic leaves trails); a long-range
        // barrier would make the whole goal unreachable. Anything further is handled by stop-and-replan.
        const double d = std::hypot(live_.cx(i) - pose_.x, live_.cy(j) - pose_.y);
        // Moving tracked objects (and their short occupancy trail) belong to the behaviour layer, not to the
        // static stop / replan logic: a truck ahead is followed, a crossing one yielded to.
        if (inMovingAgent(live_.cx(i), live_.cy(j))) continue;
        if (g.toCell(live_.cx(i), live_.cy(j), si, sj) && d > vp_.front + 1.0 && d < live_radius_) g.at(si, sj) = kLethal;
      }
    return g;
  }

  size_t progress() const {  // nearest index on the current plan, searched forward from the last one
    size_t best = prog_;
    double bd = 1e18;
    for (size_t i = prog_; i < std::min(full_.size(), prog_ + 80); ++i) {
      const double d = std::hypot(full_[i].x - pose_.x, full_[i].y - pose_.y);
      if (d < bd) bd = d, best = i;
    }
    return best;
  }

  void tick() {
    if (!have_pose_ || site_.empty()) return;
    if (have_goal_ && state_ == "planning") plan(pose_, v_ >= 0 ? (std::abs(v_) > 0.2 ? 1 : 0) : -1);
    if (state_ == "failed" && (now() - fail_t_).seconds() > 2.0) state_ = "planning";
    if (state_ == "driving") {
      prog_ = progress();
      const double dg = std::hypot(pose_.x - goal_.x, pose_.y - goal_.y);
      if (dg < 1.2 && std::abs(v_) < 0.15 && prog_ + 5 >= full_.size()) state_ = "arrived";
      checkBlocked();
      if (state_ == "driving" && !waiting_ && behave_) updateBehaviour();
    }
    publishMetrics();
  }

  void checkBlocked() {
    const CostGrid g = fused();
    Path rest;
    for (size_t i = prog_; i < full_.size(); ++i) rest.push_back({full_[i].x, full_[i].y, full_[i].yaw, full_[i].dir});
    size_t hit = SIZE_MAX;
    double s = 0;
    for (size_t i = 0; i < rest.size() && s < 45.0; ++i) {
      if (i) s += std::hypot(rest[i].x - rest[i - 1].x, rest[i].y - rest[i - 1].y);
      if (footprintCollides(g, rest[i].x, rest[i].y, rest[i].yaw, vp_, 0.15)) { hit = i; break; }
    }
    if (hit == SIZE_MAX) {
      block_n_ = 0;
      if (waiting_) waiting_ = false, publishTrajectory(full_traj_from(prog_), false), uncapped();  // cleared: resume
      return;
    }
    blocked_s_ = s;
    // Confirm a blockage over 3 ticks (0.6 s) unless it is inside the stopping distance (+5 m): a dust puff or a
    // one-scan blip far ahead should not make a 72 t truck brake.
    if (!waiting_ && ++block_n_ < 3 && s > v_ * v_ / (2 * 1.5) + 5.0) return;
    if (!waiting_) {
      waiting_ = true, wait_t0_ = now();
      Trajectory t = full_traj_from(prog_);
      const double stop_s = t.front().s + std::max(0.0, s - 3.0);  // stop 3 m before the blocked pose
      for (auto& p : t)
        if (p.s >= stop_s) p.v = 0;
      for (size_t i = t.size() - 1; i-- > 0;) {
        const double vm = std::sqrt(t[i + 1].v * t[i + 1].v + 2 * sp_.a_dec * 1.3 * (t[i + 1].s - t[i].s));
        if (std::abs(t[i].v) > vm) t[i].v = t[i].dir * vm;
      }
      stop_pose_ = {rest[hit].x, rest[hit].y, rest[hit].yaw};
      publishTrajectory(t, true);
      RCLCPP_INFO(get_logger(), "path blocked %.1f m ahead: stopping, replan in %.1f s if it stays", s, wait_s_);
    } else if ((now() - wait_t0_).seconds() > (stats_.status == "route" ? route_wait_s_ : wait_s_) && std::abs(v_) < 0.3) {
      // On a surveyed route (haul road) the usual blocker is traffic that clears: wait longer before leaving it.
      ++replans_, waiting_ = false;
      plan(pose_, 0);
    }
  }

  // Movers the behaviour layer may act on: vehicle-sized (long side), plausible speed, and consistent (position and
  // velocity agree with the previous scans, velocity smoothed over them) for >= 4 scans. The tracker also gives dust puffs and berm fragments a
  // velocity from one scan to the next; those are random and never become agents.
  void onObjects(const autoware_perception_msgs::msg::DetectedObjects& m) {
    const double dt = std::clamp((now() - agents_t_).seconds(), 0.02, 0.5);
    std::vector<std::pair<Agent, int>> cand;
    for (const auto& o : m.objects) {
      const auto& k = o.kinematics;
      if (!k.has_twist) continue;
      const auto& q = k.pose_with_covariance.pose.orientation;
      const double yaw = yawFromQuat(Eigen::Quaterniond(q.w, q.x, q.y, q.z)), c = std::cos(yaw), sn = std::sin(yaw);
      const double tx = k.twist_with_covariance.twist.linear.x, ty = k.twist_with_covariance.twist.linear.y;
      Agent a;
      a.x = k.pose_with_covariance.pose.position.x, a.y = k.pose_with_covariance.pose.position.y;
      a.vx = c * tx - sn * ty, a.vy = sn * tx + c * ty, a.length = o.shape.dimensions.x, a.width = o.shape.dimensions.y;
      a.yaw = yaw;
      const double v = std::hypot(a.vx, a.vy), big = std::max(a.length, a.width);
      if (v < bp_.moving || v > 12.0 || big < 2.0 || big > 14.0) continue;
      int n = 1;
      const std::pair<Agent, int>* match = nullptr;
      for (const auto& tr : tracks_) {
        const Agent& p = tr.first;
        if (std::hypot(p.x + p.vx * dt - a.x, p.y + p.vy * dt - a.y) < 2.5 && std::hypot(p.vx - a.vx, p.vy - a.vy) < 3.0 && tr.second + 1 > n)
          n = tr.second + 1, match = &tr;
      }
      if (match) a.vx = 0.5 * (a.vx + match->first.vx), a.vy = 0.5 * (a.vy + match->first.vy);
      cand.push_back({a, n});
    }
    tracks_ = cand, agents_.clear();
    for (const auto& [a, n] : tracks_)
      if (n >= 4) agents_.push_back(a);
    agents_t_ = now();
  }

  // Dust echoes in the lane corridor ahead of the front (ego frame; the map-frame cloud is counted straight).
  void onDust(const sensor_msgs::msg::PointCloud2& m) {
    if (!have_pose_ || m.point_step < 8) return;
    const double c = std::cos(pose_.yaw), sn = std::sin(pose_.yaw), half = vp_.width / 2 + 1.5;
    int n = 0;
    for (size_t k = 0; k + 8 <= m.data.size(); k += m.point_step) {
      float x, y;
      std::memcpy(&x, &m.data[k], 4), std::memcpy(&y, &m.data[k + 4], 4);
      const double dx = x - pose_.x, dy = y - pose_.y, u = c * dx + sn * dy, l = -sn * dx + c * dy;
      n += u > vp_.front && u < vp_.front + bp_.dust_range && std::abs(l) < half;
    }
    dust_n_ = n, dust_t_ = now();
  }

  std::vector<Agent> agentsNow() const {
    return (now() - agents_t_).seconds() < 0.6 ? agents_ : std::vector<Agent>{};
  }

  bool inMovingAgent(double x, double y) const {
    for (const auto& a : agentsNow()) {
      const double v = std::hypot(a.vx, a.vy), c = a.vx / v, sn = a.vy / v, dx = x - a.x, dy = y - a.y;
      const double u = c * dx + sn * dy, l = -sn * dx + c * dy, r = std::max(a.length, a.width) / 2 + 1.0;
      if (u < r && u > -r - 1.0 * v && std::abs(l) < r) return true;  // + 1 s trail behind it
    }
    return false;
  }

  void uncapped() { pub_name_ = "cruise", pub_q_ = 99.0; }  // the last trajectory went out without caps

  void updateBehaviour() {
    const Trajectory rest = full_traj_from(prog_);
    const int dust = (now() - dust_t_).seconds() < 0.6 ? dust_n_ : 0;
    beh_ = decideBehaviour(rest, vp_.width / 2, vp_.front, vp_.rear, std::abs(v_), agentsNow(), dust, bp_);
    const double q = beh_.v_eff > 1e8 ? 99.0 : std::round(beh_.v_eff * 2) / 2;  // 0.5 m/s steps
    if (beh_.name == pub_name_ && std::abs(q - pub_q_) < 0.5) return;
    // Tighten at once, relax at most once a second (each new trajectory resets the controller's integrator).
    if (beh_.name == pub_name_ && q > pub_q_ && (now() - pub_t_).seconds() < 1.0) return;
    pub_name_ = beh_.name, pub_q_ = q, pub_t_ = now();
    const bool stop = beh_.s_stop < 1e8 && beh_.name == "yield";
    if (stop) {
      for (const auto& p : rest)
        if (p.s >= beh_.s_stop) { stop_pose_ = {p.x, p.y, p.yaw}; break; }
    }
    publishTrajectory(applyBehaviour(rest, beh_, std::abs(v_), bp_), stop);
  }

  Trajectory full_traj_from(size_t k) const {
    if (full_.empty()) return {};
    return Trajectory(full_.begin() + long(std::min(k, full_.size() - 1)), full_.end());
  }

  void plan(const Pose2& from, int dir) {
    PlanStats st;
    Path p = planner_->plan(fused(), from, dir, goal_, &st);
    if (p.empty() && st.expansions <= 2 && tight_planner_) {  // boxed in by the safety margin: retry with half of it
      PlanStats st2;
      p = tight_planner_->plan(fused(), from, dir, goal_, &st2);
      if (!p.empty()) st = st2, st.status = "ok_tight_margin";
    }
    stats_ = st;
    if (p.empty()) {
      state_ = "failed", fail_t_ = now();
      RCLCPP_WARN(get_logger(), "planning failed: %s after %d expansions (%.0f ms)", st.status.c_str(), st.expansions, st.ms);
      if (st.expansions <= 1 && !dumped_) dumpFused(from), dumped_ = true;
      publishTrajectory({}, false);
      return;
    }
    full_ = timeParameterize(p, site_, sp_, v_);
    prog_ = 0, state_ = "driving";
    cusps_ = 0;
    for (size_t i = 1; i < p.size(); ++i) cusps_ += p[i].dir != p[i - 1].dir;
    RCLCPP_INFO(get_logger(), "planned %.1f m, %d cusps, %d expansions, RS shot %s, %.0f ms", full_.back().s, cusps_, st.expansions,
                st.rs_shot ? "yes" : "no", st.ms);
    publishTrajectory(full_, false), uncapped();
    publishSearch(p);
  }

  // Debug aid: the fused costmap (+-40 m around the start) as a PGM, live-only lethal cells marked grey.
  void dumpFused(const Pose2& from) {
    const CostGrid g = fused();
    int ci, cj;
    if (!g.toCell(from.x, from.y, ci, cj)) return;
    const int r = int(40.0 / g.res);
    const std::string dump = get_parameter("debug_dump_file").as_string();
    if (dump.empty()) return;
    std::ofstream f(dump);
    f << "P2\n" << 2 * r << " " << 2 * r << "\n255\n";
    for (int j = cj + r - 1; j >= cj - r; --j) {
      for (int i = ci - r; i < ci + r; ++i) {
        int v = 255;
        if (g.inside(i, j)) {
          const bool live = g.at(i, j) >= kLethal && site_.at(i, j) < kLethal;
          v = live ? 110 : g.at(i, j) >= kLethal ? 0 : 255 - 2 * std::max<int>(0, g.at(i, j));
        }
        if (std::abs(i - ci) < 2 && std::abs(j - cj) < 2) v = 180;
        f << v << " ";
      }
      f << "\n";
    }
    RCLCPP_WARN(get_logger(), "fused costmap around the start written to %s", dump.c_str());
  }

  void publishTrajectory(const Trajectory& t, bool stopping) {
    autoware_planning_msgs::msg::Trajectory m;
    m.header.frame_id = "map", m.header.stamp = now();
    nav_msgs::msg::Path path;
    path.header = m.header;
    double time = 0;
    for (size_t i = 0; i < t.size(); ++i) {
      const auto& p = t[i];
      if (i) {  // time_from_start from the profile (trapezoid between points)
        const double ds = std::abs(p.s - t[i - 1].s), vm = 0.5 * (std::abs(p.v) + std::abs(t[i - 1].v));
        time += ds / std::max(vm, 0.05);
      }
      autoware_planning_msgs::msg::TrajectoryPoint q;
      q.time_from_start = rclcpp::Duration::from_seconds(time);
      q.pose.position.x = p.x, q.pose.position.y = p.y, q.pose.position.z = z(p.x, p.y);
      q.pose.orientation.z = std::sin(p.yaw / 2), q.pose.orientation.w = std::cos(p.yaw / 2);
      q.longitudinal_velocity_mps = float(p.v);  // signed: < 0 = reverse (Autoware convention)
      q.acceleration_mps2 = float(p.a);
      // kappa is geometric (dyaw/ds); the road-wheel angle that drives it flips sign in reverse.
      q.heading_rate_rps = float(std::abs(p.v) * p.kappa);
      q.front_wheel_angle_rad = float(std::atan(vp_.wheelbase * p.kappa * p.dir));
      m.points.push_back(q);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = m.header, ps.pose = q.pose;
      ps.pose.position.z += 0.35;
      path.poses.push_back(ps);
    }
    traj_pub_->publish(m);
    path_pub_->publish(path);
    stopping_ = stopping;
    publishPathMarkers(t, stopping);
  }

  std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r, c.g = g, c.b = b, c.a = a;
    return c;
  }
  geometry_msgs::msg::Point pt(double x, double y, double dz = 0.4) {
    geometry_msgs::msg::Point p;
    p.x = x, p.y = y, p.z = z(x, y) + dz;
    return p;
  }

  void publishPathMarkers(const Trajectory& t, bool stopping) {
    visualization_msgs::msg::MarkerArray ma;
    Marker clear;
    clear.action = Marker::DELETEALL;
    ma.markers.push_back(clear);
    Marker line;
    line.header.frame_id = "map", line.ns = "trajectory", line.id = 0, line.type = Marker::LINE_STRIP;
    line.pose.orientation.w = 1, line.scale.x = 0.45 * viz_scale_;
    Marker fp = line;
    fp.ns = "footprints", fp.type = Marker::LINE_LIST, fp.scale.x = 0.08 * viz_scale_;
    double last_fp = -1e9;
    for (const auto& p : t) {
      const double sp = std::min(1.0, std::abs(p.v) / std::max(0.1, sp_.v_fwd));
      line.points.push_back(pt(p.x, p.y, 0.45));
      line.colors.push_back(p.dir > 0 ? rgba(0.2f, 0.6f + 0.4f * float(sp), 1.0f, 1.f) : rgba(1.0f, 0.25f, 0.85f, 1.f));
      if (p.s - last_fp > 3.0) {
        last_fp = p.s;
        const double c = std::cos(p.yaw), s = std::sin(p.yaw);
        std::array<geometry_msgs::msg::Point, 4> v;
        const double ax[4] = {vp_.front, vp_.front, -vp_.rear, -vp_.rear}, ay[4] = {vp_.width / 2, -vp_.width / 2, -vp_.width / 2, vp_.width / 2};
        for (int k = 0; k < 4; ++k) v[size_t(k)] = pt(p.x + c * ax[k] - s * ay[k], p.y + s * ax[k] + c * ay[k], 0.3);
        for (int k = 0; k < 4; ++k) {
          fp.points.push_back(v[size_t(k)]), fp.points.push_back(v[size_t((k + 1) % 4)]);
          const auto col = p.dir > 0 ? rgba(0.4f, 0.8f, 1.f, 0.55f) : rgba(1.f, 0.4f, 0.9f, 0.6f);
          fp.colors.push_back(col), fp.colors.push_back(col);
        }
      }
    }
    if (line.points.size() > 1) ma.markers.push_back(line), ma.markers.push_back(fp);
    Marker goal;
    goal.header.frame_id = "map", goal.ns = "goal", goal.id = 0, goal.type = Marker::ARROW;
    goal.pose.position = pt(goal_.x, goal_.y, 1.5);
    goal.pose.orientation.z = std::sin(goal_.yaw / 2), goal.pose.orientation.w = std::cos(goal_.yaw / 2);
    goal.scale.x = 6.0, goal.scale.y = 1.2, goal.scale.z = 1.2, goal.color = rgba(0.3f, 1.f, 0.4f, 0.9f);
    if (have_goal_) ma.markers.push_back(goal);
    if (stopping) {
      Marker bar;
      bar.header.frame_id = "map", bar.ns = "stop", bar.id = 0, bar.type = Marker::CUBE;
      bar.pose.position = pt(stop_pose_.x, stop_pose_.y, 1.2);
      bar.pose.orientation.z = std::sin(stop_pose_.yaw / 2), bar.pose.orientation.w = std::cos(stop_pose_.yaw / 2);
      bar.scale.x = 0.6, bar.scale.y = vp_.width + 2.0, bar.scale.z = 2.4, bar.color = rgba(1.f, 0.15f, 0.1f, 0.75f);
      ma.markers.push_back(bar);
    }
    viz_pub_->publish(ma);
  }

  void publishSearch(const Path& p) {
    visualization_msgs::msg::MarkerArray ma;
    Marker tree;
    tree.header.frame_id = "map", tree.ns = "search_tree", tree.id = 0, tree.type = Marker::LINE_LIST;
    tree.pose.orientation.w = 1, tree.scale.x = 0.07 * viz_scale_;
    for (const auto& e : stats_.tree) {
      tree.points.push_back(pt(e.first.x, e.first.y, 0.25)), tree.points.push_back(pt(e.second.x, e.second.y, 0.25));
      const auto c = e.second.dir > 0 ? rgba(0.55f, 0.7f, 0.95f, 0.45f) : rgba(0.95f, 0.55f, 0.9f, 0.45f);
      tree.colors.push_back(c), tree.colors.push_back(c);
    }
    if (!tree.points.empty()) ma.markers.push_back(tree);
    if (stats_.rs_shot && stats_.rs_begin < p.size()) {
      Marker shot;
      shot.header.frame_id = "map", shot.ns = "rs_shot", shot.id = 0, shot.type = Marker::LINE_STRIP;
      shot.pose.orientation.w = 1, shot.scale.x = 0.7 * viz_scale_, shot.color = rgba(1.f, 0.85f, 0.15f, 0.95f);
      for (size_t i = stats_.rs_begin ? stats_.rs_begin - 1 : 0; i < p.size(); ++i) shot.points.push_back(pt(p[i].x, p[i].y, 0.55));
      ma.markers.push_back(shot);
    }
    search_ = ma;
    viz_search_pub_ = viz_search_pub_ ? viz_search_pub_
                                      : create_publisher<visualization_msgs::msg::MarkerArray>("/planning/search", rclcpp::QoS(1).transient_local().reliable());
    viz_search_pub_->publish(ma);
  }

  void publishMetrics() {
    std::ostringstream js;
    js.precision(3);
    const double remaining = full_.empty() ? 0.0 : full_.back().s - full_[std::min(prog_, full_.size() - 1)].s;
    js << std::fixed << "{\"state\":\"" << (waiting_ ? "stopped_for_obstacle" : state_) << "\",\"plan_ms\":" << stats_.ms
       << ",\"expansions\":" << stats_.expansions << ",\"rs_shot\":" << (stats_.rs_shot ? "true" : "false")
       << ",\"status\":\"" << stats_.status << "\",\"length\":" << (full_.empty() ? 0.0 : full_.back().s) << ",\"cusps\":" << cusps_
       << ",\"replans\":" << replans_ << ",\"remaining\":" << remaining << ",\"blocked_at\":" << (waiting_ ? blocked_s_ : -1.0)
       << ",\"goal\":[" << goal_.x << "," << goal_.y << "," << goal_.yaw << "]"
       << ",\"behaviour\":\"" << (waiting_ ? "stop" : beh_.name) << "\",\"behaviour_v\":" << (beh_.v_eff > 1e8 ? -1.0 : beh_.v_eff)
       << ",\"behaviour_info\":" << beh_.info << ",\"dust_echoes\":" << dust_n_ << ",\"moving_objects\":" << agentsNow().size() << "}";
    std_msgs::msg::String s;
    s.data = js.str();
    metrics_pub_->publish(s);
  }

  VehicleParams vp_;
  PlannerParams pp_;
  SpeedParams sp_;
  BehaviourParams bp_;
  Behaviour beh_;
  std::vector<Agent> agents_;
  std::vector<std::pair<Agent, int>> tracks_;  // mover candidates and how many consistent scans they have
  int dust_n_ = 0, block_n_ = 0;
  bool behave_ = true;
  std::string pub_name_ = "cruise";
  double pub_q_ = 99.0;
  rclcpp::Time agents_t_{0, 0, RCL_ROS_TIME}, dust_t_{0, 0, RCL_ROS_TIME}, pub_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr dust_sub_;
  std::unique_ptr<HybridAStar> planner_, tight_planner_;
  double route_wait_s_ = 25.0, viz_scale_ = 1.0;
  CostGrid site_, live_;
  Grid<float> elev_;
  Pose2 pose_, goal_, stop_pose_;
  double v_ = 0, wait_s_ = 2.5, blocked_s_ = 0, live_radius_ = 40.0;
  bool have_pose_ = false, have_goal_ = false, use_live_ = true, waiting_ = false, stopping_ = false, dumped_ = false;
  std::string state_ = "idle";
  Trajectory full_;  // current plan; prog_ indexes it
  size_t prog_ = 0;
  int cusps_ = 0, replans_ = 0;
  PlanStats stats_;
  rclcpp::Time wait_t0_, fail_t_;
  visualization_msgs::msg::MarkerArray search_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_, viz_search_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, live_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace iad::planning

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iad::planning::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
