// LiDAR perception for unstructured sites:
//   range image -> self/range crop -> dust filter -> ground segmentation (gravity-aligned) ->
//   overhang split -> clustering + boxes + tracking -> log-odds occupancy grid (map frame) for planning.
// mode:=naive runs the baseline (no dust filter, one RANSAC plane in the sensor frame) for comparison.
//
// In : <input_topic> sensor_msgs/PointCloud2, TF base_link<-lidar (static) and map<-base_link
// Out: ~/points/{ground,obstacle,dust,overhang} + ~/points/classified (class field), PointCloud2, map frame
//      ~/objects        autoware_perception_msgs/DetectedObjects (tracked boxes with velocity)
//      ~/objects/markers visualization_msgs/MarkerArray
//      ~/occupancy_grid nav_msgs/OccupancyGrid (rolling window around the vehicle)
//      ~/metrics        std_msgs/String JSON (per-frame counts and timing)
// Ground-truth scoring lives outside this package (iad_eval): this node never sees simulator labels.
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <deque>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "iad_perception/dust_filter.hpp"
#include "iad_perception/ground_segmentation.hpp"
#include "iad_perception/objects.hpp"
#include "iad_perception/occupancy.hpp"
#include "iad_perception/types.hpp"

namespace iad::perception {
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

enum Cat : uint8_t { kCatGround = 0, kCatObstacle = 1, kCatDust = 2, kCatOverhang = 3, kCatSelf = 4 };

class PerceptionNode : public rclcpp::Node {
 public:
  explicit PerceptionNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("perception", opt) {
    mode_ = declare_parameter("mode", "ours");
    const std::string input = declare_parameter("input_topic", "/sensing/lidar/top/points");
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    // ego body (base_link = rear axle centre on the ground) for the self-filter
    front_ = declare_parameter("vehicle.front", 6.6), rear_ = declare_parameter("vehicle.rear", 2.8);
    width_ = declare_parameter("vehicle.width", 4.2), height_ = declare_parameter("vehicle.height", 4.4);
    max_range_ = declare_parameter("max_range", 80.0);
    overhang_ = declare_parameter("overhang_height", height_ + 1.0);
    DustParams dp;
    dp.low_intensity = declare_parameter("dust.low_intensity", 12.0);
    dp.very_low_intensity = declare_parameter("dust.very_low_intensity", 4.0);
    low_intensity_ = dp.low_intensity;
    max_weak_fraction_ = declare_parameter("dust.max_weak_cluster_fraction", 0.6);
    dp.use_temporal = declare_parameter("dust.use_temporal", true);
    dp.along_surface = int(declare_parameter("dust.along_surface", 4));
    dust_ = DustFilter(dp);
    LogOddsGrid::Params gp;
    gp.size = declare_parameter("grid.size", 140.0);
    gp.res = declare_parameter("grid.resolution", 0.5);
    grid_ = std::make_unique<LogOddsGrid>(gp);
    ours_ = mode_ != "naive";

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    pub_ground_ = create_publisher<PointCloud2>("~/points/ground", qos);
    pub_obst_ = create_publisher<PointCloud2>("~/points/obstacle", qos);
    pub_dust_ = create_publisher<PointCloud2>("~/points/dust", qos);
    pub_over_ = create_publisher<PointCloud2>("~/points/overhang", qos);
    pub_cls_ = create_publisher<PointCloud2>("~/points/classified", rclcpp::QoS(5));
    pub_objects_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>("~/objects", 5);
    pub_markers_ = create_publisher<MarkerArray>("~/objects/markers", 5);
    pub_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid", 2);
    pub_metrics_ = create_publisher<std_msgs::msg::String>("~/metrics", 10);
    sub_ = create_subscription<PointCloud2>(input, qos, [this](PointCloud2::ConstSharedPtr m) { onCloud(*m); });
    RCLCPP_INFO(get_logger(), "perception mode=%s input=%s", mode_.c_str(), input.c_str());
  }

 private:
  void onCloud(const PointCloud2& msg) {
    Stopwatch total;
    Eigen::Isometry3d T_bl, T_mb;
    try {
      T_bl = tf2::transformToEigen(tf_buffer_->lookupTransform(base_frame_, msg.header.frame_id, tf2::TimePointZero));
      T_mb = tf2::transformToEigen(tf_buffer_->lookupTransform(map_frame_, base_frame_, msg.header.stamp,
                                                               rclcpp::Duration::from_seconds(0.1)));
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", e.what());
      return;
    }
    const Scan scan = scanFromMsg(msg);
    const auto& c = scan.pts;
    const size_t n = c.size();
    const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(T_mb.linear()));
    const Eigen::Matrix3d R_lb = (Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    Eigen::Isometry3d T_ml = Eigen::Isometry3d::Identity();  // map <- level (gravity-aligned, yaw-following)
    T_ml.translation() = T_mb.translation();
    T_ml.linear() = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    std::vector<Eigen::Vector3f> P(n), S(n), M(n);
    std::vector<uint8_t> self(n, 0);
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d pl(c[i].x, c[i].y, c[i].z), pb = T_bl * pl;
      self[i] = (pb.x() > -rear_ - 0.6 && pb.x() < front_ + 0.6 && std::abs(pb.y()) < width_ / 2 + 0.6 &&
                 pb.z() < height_ + 1.2) || pl.norm() > max_range_;
      P[i] = (R_lb * pb).cast<float>(), S[i] = pl.cast<float>();
      M[i] = (T_ml * P[i].cast<double>()).cast<float>();
    }
    // 1. dust
    Stopwatch sw;
    const std::vector<uint8_t> dust = ours_ ? dust_.classify(scan, M) : std::vector<uint8_t>(n, 0);
    const double ms_dust = sw.ms();
    std::vector<uint8_t> ignore(n);
    for (size_t i = 0; i < n; ++i) ignore[i] = self[i] || dust[i];
    // 2. ground
    sw.reset();
    const GroundResult gr = ours_ ? segmentGroundCzm(P, ignore, czm_, R_lb * Eigen::Vector3d::UnitZ())
                                  : segmentGroundRansac(S, self, 0.25, 150, 42);
    const double ms_ground = sw.ms();
    std::vector<uint8_t> cat(n);
    std::vector<int> obst_idx;
    for (size_t i = 0; i < n; ++i) {
      if (self[i]) cat[i] = kCatSelf;
      else if (dust[i]) cat[i] = kCatDust;
      else if (gr.ground[i]) cat[i] = kCatGround;
      else if (ours_ && gr.height[i] > overhang_) cat[i] = kCatOverhang;
      else cat[i] = kCatObstacle, obst_idx.push_back(int(i));
    }
    // 3. objects (level frame) -> map frame + tracking
    sw.reset();
    std::vector<float> h0(n, 0.f);
    for (size_t i = 0; i < n; ++i) h0[i] = P[i].z() - gr.height[i];  // local ground elevation under the point
    std::vector<uint8_t> in_cluster;
    std::vector<uint8_t> weak(n, 0);
    for (size_t i = 0; i < n; ++i) weak[i] = c[i].intensity < low_intensity_;
    std::vector<Obb> boxes = clusterObstacles(P, obst_idx, h0, 0.5, ours_ ? 6 : 1, &in_cluster, ours_ ? &weak : nullptr,
                                              max_weak_fraction_);
    const double yaw = rpy.z();
    for (auto& b : boxes) {
      const Eigen::Vector3d cm = T_ml * Eigen::Vector3d(b.c.x(), b.c.y(), 0.0);
      b.c = cm.head<2>(), b.yaw = wrapAngle(b.yaw + yaw);
      b.z0 += T_ml.translation().z(), b.z1 += T_ml.translation().z();
    }
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    tracker_.update(boxes, t);
    const double ms_cluster = sw.ms();
    {  // debug: near-ring (9-12 m) obstacle points and their height above the ground model
      ring_dbg_n_ = 0, ring_dbg_obst_ = 0, ring_dbg_h_ = 0, ring_dbg_ring_ = -1;
      for (size_t i = 0; i < n; ++i) {
        const double r = std::sqrt(double(S[i].squaredNorm()));
        if (r < 9.0 || r > 12.0 || self[i] || dust[i]) continue;
        ++ring_dbg_n_;
        if (cat[i] == kCatObstacle) ++ring_dbg_obst_, ring_dbg_h_ += gr.height[i], ring_dbg_ring_ = c[i].ring;
      }
      if (ring_dbg_obst_) ring_dbg_h_ /= ring_dbg_obst_;
    }
    // 4. occupancy (map frame): only cluster-validated obstacle points mark cells, so residual dust
    // specks never block the planner; ground returns trace free space.
    sw.reset();
    const Eigen::Vector3d origin = T_mb * T_bl.translation();
    grid_->recenter(origin.x(), origin.y());
    std::vector<Eigen::Vector2d> hits, frees;
    for (size_t i = 0; i < n; ++i) {
      if (cat[i] == kCatObstacle && in_cluster[i]) hits.push_back(M[i].head<2>().cast<double>());
      else if (cat[i] == kCatGround && (i % 2 == 0)) frees.push_back(M[i].head<2>().cast<double>());
    }
    grid_->integrate(origin.head<2>(), hits, frees);
    const double ms_grid = sw.ms();
    const double ms_total = total.ms();

    ego_xy_ = T_mb.translation().head<2>();
    publishClouds(msg.header.stamp, scan, M, cat);
    publishObjects(msg.header.stamp);
    publishGrid(msg.header.stamp);
    publishMetrics(t, cat, boxes.size(), ms_total, ms_dust, ms_ground, ms_cluster, ms_grid);
  }

  void publishClouds(const builtin_interfaces::msg::Time& st, const Scan& s, const std::vector<Eigen::Vector3f>& M,
                     const std::vector<uint8_t>& cat) {
    std::array<std::vector<Eigen::Vector3f>, 4> p;
    std::array<std::vector<float>, 4> in;
    std::vector<Eigen::Vector3f> all;
    std::vector<float> all_i;
    std::vector<uint8_t> all_c;
    for (size_t i = 0; i < M.size(); ++i) {
      if (cat[i] >= kCatSelf) continue;
      p[cat[i]].push_back(M[i]), in[cat[i]].push_back(s.pts[i].intensity);
      all.push_back(M[i]), all_i.push_back(s.pts[i].intensity), all_c.push_back(cat[i]);
    }
    pub_ground_->publish(cloudToMsg(p[kCatGround], in[kCatGround], map_frame_, st));
    pub_obst_->publish(cloudToMsg(p[kCatObstacle], in[kCatObstacle], map_frame_, st));
    pub_dust_->publish(cloudToMsg(p[kCatDust], in[kCatDust], map_frame_, st));
    pub_over_->publish(cloudToMsg(p[kCatOverhang], in[kCatOverhang], map_frame_, st));
    if (pub_cls_->get_subscription_count() > 0) pub_cls_->publish(cloudToMsg(all, all_i, map_frame_, st, &all_c));
  }

  static uint8_t autowareLabel(const Obb& b) {
    using C = autoware_perception_msgs::msg::ObjectClassification;
    if (b.cls == "person") return C::PEDESTRIAN;
    if (b.cls == "vehicle") return b.length > 7.0 ? C::TRUCK : C::CAR;
    if (b.cls == "machine") return C::TRUCK;
    return C::UNKNOWN;
  }

  void publishObjects(const builtin_interfaces::msg::Time& st) {
    autoware_perception_msgs::msg::DetectedObjects objs;
    objs.header.frame_id = map_frame_, objs.header.stamp = st;
    MarkerArray ma;
    Marker clear;
    clear.action = Marker::DELETEALL;
    ma.markers.push_back(clear);
    Marker lines;
    lines.header = objs.header;
    lines.ns = "boxes", lines.id = 0, lines.type = Marker::LINE_LIST, lines.action = Marker::ADD;
    lines.pose.orientation.w = 1.0, lines.scale.x = 0.12;
    int id = 1;
    for (const auto& tr : tracker_.tracks()) {
      if (!tr.confirmed() || tr.misses > 0) continue;
      const Obb& b = tr.box;
      // ---- interface message
      autoware_perception_msgs::msg::DetectedObject o;
      o.existence_probability = float(std::min(1.0, 0.5 + 0.05 * tr.hits));
      autoware_perception_msgs::msg::ObjectClassification oc;
      oc.label = autowareLabel(b), oc.probability = 0.8f;
      o.classification.push_back(oc);
      auto& pose = o.kinematics.pose_with_covariance.pose;
      pose.position.x = b.c.x(), pose.position.y = b.c.y(), pose.position.z = 0.5 * (b.z0 + b.z1);
      pose.orientation.z = std::sin(b.yaw / 2), pose.orientation.w = std::cos(b.yaw / 2);
      o.kinematics.orientation_availability = autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      o.kinematics.has_twist = tr.hits > 5;
      const double cy = std::cos(b.yaw), sy = std::sin(b.yaw);  // twist in the object frame
      o.kinematics.twist_with_covariance.twist.linear.x = cy * tr.v.x() + sy * tr.v.y();
      o.kinematics.twist_with_covariance.twist.linear.y = -sy * tr.v.x() + cy * tr.v.y();
      o.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      o.shape.dimensions.x = b.length, o.shape.dimensions.y = b.width, o.shape.dimensions.z = b.z1 - b.z0;
      objs.objects.push_back(o);
      // ---- RViz
      std_msgs::msg::ColorRGBA col;
      col.a = 1.0;
      if (b.cls == "berm") col.r = 0.75, col.g = 0.55, col.b = 0.35, col.a = 0.5;
      else if (b.cls == "person") col.r = 1.0, col.g = 0.2, col.b = 0.6;
      else if (b.cls == "vehicle") col.r = 0.2, col.g = 0.8, col.b = 1.0;
      else if (b.cls == "machine") col.r = 1.0, col.g = 0.75, col.b = 0.1;
      else col.r = 1.0, col.g = 0.35, col.b = 0.2;
      std::array<geometry_msgs::msg::Point, 8> v;
      for (int k = 0; k < 8; ++k) {
        const double lx = (k & 1 ? 0.5 : -0.5) * b.length, ly = (k & 2 ? 0.5 : -0.5) * b.width;
        v[size_t(k)].x = b.c.x() + cy * lx - sy * ly, v[size_t(k)].y = b.c.y() + sy * lx + cy * ly;
        v[size_t(k)].z = k & 4 ? b.z1 : b.z0;
      }
      static const int E[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
      for (const auto& e : E) {
        lines.points.push_back(v[size_t(e[0])]), lines.points.push_back(v[size_t(e[1])]);
        lines.colors.push_back(col), lines.colors.push_back(col);
      }
      const double dist = (b.c - ego_xy_).norm();
      if (b.cls == "berm" || (b.cls == "obstacle" && b.z1 - b.z0 < 0.8) || dist > 45.0 || b.n < 15) continue;  // label clutter
      Marker txt;
      txt.header = lines.header, txt.ns = "labels", txt.id = id++, txt.type = Marker::TEXT_VIEW_FACING;
      txt.pose.position.x = b.c.x(), txt.pose.position.y = b.c.y(), txt.pose.position.z = b.z1 + 1.2;
      txt.pose.orientation.w = 1.0, txt.scale.z = 0.95, txt.color = col;
      std::ostringstream os;
      os.precision(1);
      os << std::fixed << b.cls << " #" << tr.id;
      if (tr.speed() > 1.0 && tr.hits > 5) os << "  " << tr.speed() << " m/s";
      txt.text = os.str();
      ma.markers.push_back(txt);
      if (tr.speed() > 1.0 && tr.hits > 5) {  // velocity arrow
        Marker ar;
        ar.header = lines.header, ar.ns = "velocity", ar.id = id++, ar.type = Marker::ARROW;
        geometry_msgs::msg::Point a, bb;
        a.x = b.c.x(), a.y = b.c.y(), a.z = b.z1 + 0.3;
        bb.x = a.x + tr.v.x() * 1.5, bb.y = a.y + tr.v.y() * 1.5, bb.z = a.z;
        ar.points = {a, bb};
        ar.scale.x = 0.35, ar.scale.y = 0.8, ar.scale.z = 0.8, ar.color = col;
        ma.markers.push_back(ar);
      }
    }
    ma.markers.push_back(lines);
    pub_objects_->publish(objs);
    pub_markers_->publish(ma);
  }

  void publishGrid(const builtin_interfaces::msg::Time& st) {
    const CostGrid g = grid_->toCostGrid();
    nav_msgs::msg::OccupancyGrid og;
    og.header.frame_id = map_frame_, og.header.stamp = st;
    og.info.resolution = float(g.res), og.info.width = uint32_t(g.w), og.info.height = uint32_t(g.h);
    og.info.origin.position.x = g.ox, og.info.origin.position.y = g.oy, og.info.origin.orientation.w = 1.0;
    og.data.assign(g.data.begin(), g.data.end());
    pub_grid_->publish(og);
  }

  void publishMetrics(double t, const std::vector<uint8_t>& cat, size_t n_boxes, double ms, double ms_dust,
                      double ms_ground, double ms_cluster, double ms_grid) {
    std::array<size_t, 5> cnt{};
    for (uint8_t k : cat) cnt[k]++;
    size_t occ = 0;
    for (size_t k = 0; k < grid_->size(); ++k) occ += grid_->occupied(k);
    recent_ms_.push_back(ms);
    if (recent_ms_.size() > 50) recent_ms_.pop_front();
    std::vector<double> sorted(recent_ms_.begin(), recent_ms_.end());
    std::nth_element(sorted.begin(), sorted.begin() + long(sorted.size() / 2), sorted.end());
    ++frames_, sum_ms_ += ms;
    std::ostringstream js;
    js.precision(4);
    js << std::fixed << "{\"t\":" << t << ",\"mode\":\"" << mode_ << "\",\"points\":" << cat.size()
       << ",\"ground\":" << cnt[kCatGround] << ",\"obstacle\":" << cnt[kCatObstacle] << ",\"dust\":" << cnt[kCatDust]
       << ",\"overhang\":" << cnt[kCatOverhang] << ",\"self\":" << cnt[kCatSelf] << ",\"objects\":" << n_boxes
       << ",\"occupied_cells\":" << occ << ",\"ms\":" << ms << ",\"median_ms\":" << sorted[sorted.size() / 2]
       << ",\"avg_ms\":" << sum_ms_ / double(frames_) << ",\"ms_dust\":" << ms_dust << ",\"ms_ground\":" << ms_ground
       << ",\"ms_cluster\":" << ms_cluster << ",\"ms_grid\":" << ms_grid << ",\"frames\":" << frames_
       << ",\"dbg_ring_n\":" << ring_dbg_n_ << ",\"dbg_ring_obst\":" << ring_dbg_obst_ << ",\"dbg_ring_h\":" << ring_dbg_h_
       << ",\"dbg_ring\":" << ring_dbg_ring_ << "}";
    std_msgs::msg::String s;
    s.data = js.str();
    pub_metrics_->publish(s);
  }

  std::string mode_, map_frame_, base_frame_;
  bool ours_ = true;
  double front_, rear_, width_, height_, max_range_, overhang_, low_intensity_ = 12.0, max_weak_fraction_ = 0.6;
  DustFilter dust_;
  CzmParams czm_;
  std::unique_ptr<LogOddsGrid> grid_;
  Tracker tracker_;
  Eigen::Vector2d ego_xy_ = Eigen::Vector2d::Zero();
  std::deque<double> recent_ms_;
  size_t frames_ = 0, ring_dbg_n_ = 0, ring_dbg_obst_ = 0;
  double ring_dbg_h_ = 0;
  int ring_dbg_ring_ = -1;
  double sum_ms_ = 0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ground_, pub_obst_, pub_dust_, pub_over_, pub_cls_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_metrics_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

}  // namespace iad::perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iad::perception::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
