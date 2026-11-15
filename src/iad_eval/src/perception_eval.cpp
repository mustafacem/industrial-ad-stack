// perception_eval: scores iad_perception against ground truth derived from the simulated world geometry.
//
// gpu_lidar returns carry no labels, so every classified point is labelled here by where it lies: on the
// terrain surface (ground, or steep terrain), on/in a static structure, on a moving actor (poses from
// Gazebo), or nowhere = airborne dust. Occupied cells of the occupancy grid with no real structure under
// them are "phantom" cells (what dust or ground mistaken for obstacles does to the planner).
//   in : <ns>/points/classified (PointCloud2 + uint8 class: 0 ground, 1 obstacle, 2 dust, 3 overhang)
//        <ns>/occupancy_grid, /sim/traffic/poses (TFMessage), /sim/ground_truth/odom
//   out: /eval/<name> (JSON), optional JSONL log
#include <fstream>
#include <map>
#include <unordered_map>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "iad_gz_sim/world.hpp"

using namespace iad;
using namespace iad::sim;

enum Gt : int { kGtGround = 0, kGtStatic, kGtDynamic, kGtDust, kGtSteep, kGtEgo, kGtN };

class PerceptionEval : public rclcpp::Node {
 public:
  PerceptionEval() : Node("perception_eval") {
    const std::string scenario = declare_parameter("scenario", "quarry");
    world_ = makeWorld(scenario, uint32_t(declare_parameter("seed", 7)));
    const std::string ns = declare_parameter("perception_ns", "/perception");
    name_ = declare_parameter("name", "perception");
    tol_ = declare_parameter("surface_tolerance", 0.35);
    ego_front_ = declare_parameter("ego.front", 6.6), ego_rear_ = declare_parameter("ego.rear", 2.8);
    ego_width_ = declare_parameter("ego.width", 4.2);
    log_path_ = declare_parameter("log", "");
    for (size_t i = 0; i < world_.actors.size(); ++i) actor_index_[world_.actors[i].kind + "_" + std::to_string(i)] = i;
    pub_ = create_publisher<std_msgs::msg::String>("/eval/" + name_, 10);
    cls_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(ns + "/points/classified", rclcpp::QoS(5),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onCloud(*m); });
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(ns + "/occupancy_grid", 2,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { grid_ = *m, have_grid_ = true; });
    pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/sim/traffic/poses", 50, [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
      for (const auto& t : m->transforms) {
        auto it = actor_index_.find(t.child_frame_id);
        if (it == actor_index_.end()) continue;
        const auto& q = t.transform.rotation;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
        T.translation() = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        actor_pose_[it->second] = T;
      }
    });
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>("/sim/ground_truth/odom", 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      const auto& q = m->pose.pose.orientation;
      ego_ = Eigen::Isometry3d::Identity();
      ego_.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
      ego_.translation() = Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
      have_ego_ = true;
    });
    static_boxes_ = world_.boxes;
    static_boxes_.insert(static_boxes_.end(), world_.visuals.begin(), world_.visuals.end());
    for (size_t i = 0; i < static_boxes_.size(); ++i) bucketize(static_boxes_[i].c.x(), static_boxes_[i].c.y(), static_boxes_[i].radius(), int(i), box_buckets_);
    for (size_t i = 0; i < world_.cylinders.size(); ++i) bucketize(world_.cylinders[i].x, world_.cylinders[i].y, world_.cylinders[i].r, int(i), cyl_buckets_);
    RCLCPP_INFO(get_logger(), "perception_eval on %s: %zu static boxes, %zu cylinders, %zu actors", scenario.c_str(),
                static_boxes_.size(), world_.cylinders.size(), world_.actors.size());
  }

  ~PerceptionEval() override {
    if (frames_) std::printf("%s\n", summary().c_str());
  }

 private:
  static bool inBox(const Box& b, const Eigen::Vector3d& p, double m) {
    const double c = std::cos(b.yaw), s = std::sin(b.yaw);
    const Eigen::Vector3d d = p - b.c;
    const double lx = c * d.x() + s * d.y(), ly = -s * d.x() + c * d.y();
    return std::abs(lx) <= b.half.x() + m && std::abs(ly) <= b.half.y() + m && std::abs(d.z()) <= b.half.z() + m;
  }

  static constexpr double kBucket = 8.0;
  static int64_t bkey(int i, int j) { return (int64_t(i) << 32) ^ int64_t(uint32_t(j)); }
  static void bucketize(double x, double y, double r, int idx, std::unordered_map<int64_t, std::vector<int>>& b) {
    r += 0.5;
    for (int i = int(std::floor((x - r) / kBucket)); i <= int(std::floor((x + r) / kBucket)); ++i)
      for (int j = int(std::floor((y - r) / kBucket)); j <= int(std::floor((y + r) / kBucket)); ++j) b[bkey(i, j)].push_back(idx);
  }
  const std::vector<int>* bucket(const std::unordered_map<int64_t, std::vector<int>>& b, double x, double y) const {
    auto it = b.find(bkey(int(std::floor(x / kBucket)), int(std::floor(y / kBucket))));
    return it == b.end() ? nullptr : &it->second;
  }

  int label(const Eigen::Vector3d& p) const {
    if (have_ego_) {  // own body (perception self-filters it; excluded from the scores)
      const Eigen::Vector3d l = ego_.inverse() * p;
      if (l.x() > -ego_rear_ - 0.5 && l.x() < ego_front_ + 0.5 && std::abs(l.y()) < ego_width_ / 2 + 0.5 && l.z() < 6.0) return kGtEgo;
    }
    for (const auto& [i, T] : actor_pose_) {
      const Eigen::Vector3d l = T.inverse() * p;
      if (l.head<2>().norm() > 25.0) continue;
      for (const auto& part : world_.actors[i].parts)
        if (inBox(part, l, tol_)) return kGtDynamic;
    }
    const Terrain& t = world_.terrain;
    const double h = t.inside(p.x(), p.y()) ? t.height(p.x(), p.y()) : 0.0;
    if (const auto* bb = bucket(box_buckets_, p.x(), p.y()))
      for (int i : *bb)
        if (inBox(static_boxes_[size_t(i)], p, tol_)) return kGtStatic;
    if (const auto* cb = bucket(cyl_buckets_, p.x(), p.y()))
      for (int i : *cb) {
        const auto& c = world_.cylinders[size_t(i)];
        if (std::hypot(p.x() - c.x, p.y() - c.y) < c.r + tol_ && p.z() > c.z0 - tol_ && p.z() < c.z1 + tol_) return kGtStatic;
      }
    if (std::abs(p.z() - h) < tol_ || (p.z() < h && h - p.z() < 2.0)) return t.steep(p.x(), p.y()) ? kGtSteep : kGtGround;
    return kGtDust;
  }

  bool realObstacleInCell(double cx, double cy, double res) const {
    const Terrain& t = world_.terrain;
    for (int a = -1; a <= 1; ++a)
      for (int b = -1; b <= 1; ++b) {
        const double x = cx + a * res * 0.5, y = cy + b * res * 0.5;
        if (t.inside(x, y) && t.steep(x, y)) return true;
        const double h = t.inside(x, y) ? t.height(x, y) : 0.0;
        for (double z = h + 0.4; z < h + 4.5; z += 0.7) {
          const int l = label(Eigen::Vector3d(x, y, z));
          if (l == kGtStatic || l == kGtDynamic || l == kGtEgo) return true;
        }
      }
    return false;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2& m) {
    int ox = -1, oy = -1, oz = -1, oc = -1, oi = -1;
    for (const auto& f : m.fields) {
      if (f.name == "intensity") oi = int(f.offset);
      if (f.name == "x") ox = int(f.offset);
      else if (f.name == "y") oy = int(f.offset);
      else if (f.name == "z") oz = int(f.offset);
      else if (f.name == "class") oc = int(f.offset);
    }
    if (ox < 0 || oc < 0) return;
    // confusion[gt][pred], pred: 0 ground, 1 obstacle, 2 dust, 3 overhang
    std::array<std::array<long, 4>, kGtN> cm{};
    const size_t n = size_t(m.width) * size_t(m.height);
    for (size_t k = 0; k < n; ++k) {
      const uint8_t* d = &m.data[k * m.point_step];
      float x, y, z;
      std::memcpy(&x, d + ox, 4), std::memcpy(&y, d + oy, 4), std::memcpy(&z, d + oz, 4);
      const int pred = d[oc];
      if (pred > 3) continue;
      const int gt = label(Eigen::Vector3d(x, y, z));
      cm[size_t(gt)][size_t(pred)]++;
      if (oi >= 0) {  // intensity histogram per truth class (dust vs surfaces), for tuning / the write-up
        float in;
        std::memcpy(&in, d + oi, 4);
        hist_[gt == kGtDust ? 0 : 1][size_t(std::clamp(int(in / 5.0f), 0, 39))]++;
      }
    }
    for (int g = 0; g < kGtN; ++g)
      for (int p = 0; p < 4; ++p) total_[size_t(g)][size_t(p)] += cm[size_t(g)][size_t(p)];
    auto row = [&](const auto& c, int g) { return c[size_t(g)][0] + c[size_t(g)][1] + c[size_t(g)][2] + c[size_t(g)][3]; };
    auto col = [&](const auto& c, int p) {
      long s = 0;
      for (int g = 0; g < kGtN; ++g)
        if (g != kGtEgo) s += c[size_t(g)][size_t(p)];
      return s;
    };
    auto ratio = [](double a, double b) { return b > 0 ? a / b : 0.0; };
    const long gt_dust = row(cm, kGtDust), gt_ground = row(cm, kGtGround);
    const long gt_obst = row(cm, kGtStatic) + row(cm, kGtDynamic);
    const double dust_recall = ratio(cm[kGtDust][2], gt_dust), dust_prec = ratio(cm[kGtDust][2], col(cm, 2));
    const double ground_recall = ratio(cm[kGtGround][0], gt_ground), ground_prec = ratio(cm[kGtGround][0], col(cm, 0));
    const double obst_recall = ratio(cm[kGtStatic][1] + cm[kGtStatic][3] + cm[kGtDynamic][1] + cm[kGtDynamic][3], gt_obst);
    const long false_obst = cm[kGtGround][1] + cm[kGtDust][1];
    // phantom occupied cells
    long occ = 0, phantom = 0;
    if (have_grid_ && ++grid_div_ % 2 == 0) {
      const auto& g = grid_;
      for (uint32_t j = 0; j < g.info.height; ++j)
        for (uint32_t i = 0; i < g.info.width; ++i) {
          if (g.data[size_t(j) * g.info.width + i] < 100) continue;
          ++occ;
          const double cx = g.info.origin.position.x + (i + 0.5) * g.info.resolution;
          const double cy = g.info.origin.position.y + (j + 0.5) * g.info.resolution;
          phantom += !realObstacleInCell(cx, cy, g.info.resolution);
        }
      last_phantom_ = phantom, last_occ_ = occ;
      sum_phantom_ += double(phantom), ++grid_frames_;
    }
    ++frames_;
    sum_false_obst_ += double(false_obst), sum_ground_recall_ += ground_recall;
    if (gt_dust > 50) sum_dust_recall_ += dust_recall, sum_dust_prec_ += dust_prec, ++dusty_frames_;
    std::ostringstream js;
    js.precision(4);
    js << std::fixed << "{\"t\":" << rclcpp::Time(m.header.stamp).seconds() << ",\"gt_dust\":" << gt_dust
       << ",\"dust_recall\":" << dust_recall << ",\"dust_precision\":" << dust_prec << ",\"ground_recall\":" << ground_recall
       << ",\"ground_precision\":" << ground_prec << ",\"obstacle_recall\":" << obst_recall
       << ",\"false_obstacle_pts\":" << false_obst << ",\"phantom_cells\":" << last_phantom_ << ",\"occupied_cells\":" << last_occ_
       << ",\"frames\":" << frames_ << ",\"avg_false_obstacle_pts\":" << sum_false_obst_ / double(frames_)
       << ",\"avg_phantom_cells\":" << (grid_frames_ ? sum_phantom_ / double(grid_frames_) : 0.0)
       << ",\"avg_ground_recall\":" << sum_ground_recall_ / double(frames_)
       << ",\"avg_dust_recall_dusty\":" << (dusty_frames_ ? sum_dust_recall_ / double(dusty_frames_) : 0.0)
       << ",\"avg_dust_precision_dusty\":" << (dusty_frames_ ? sum_dust_prec_ / double(dusty_frames_) : 0.0)
       << ",\"dusty_frames\":" << dusty_frames_ << ",\"confusion\":[";
    for (int g = 0; g < kGtN - 1; ++g) {
      js << (g ? "," : "") << "[";
      for (int p = 0; p < 4; ++p) js << (p ? "," : "") << cm[size_t(g)][size_t(p)];
      js << "]";
    }
    js << "]}";
    std_msgs::msg::String s;
    s.data = js.str();
    pub_->publish(s);
    if (!log_path_.empty()) {
      if (!log_.is_open()) log_.open(log_path_);
      log_ << s.data << "\n";
    }
  }

  std::string summary() const {
    std::ostringstream o;
    o.precision(3);
    o << "intensity histogram (5-unit bins) dust:";
    for (long v : hist_[0]) o << " " << v;
    o << " | surfaces:";
    for (long v : hist_[1]) o << " " << v;
    o << "\n";
    o << name_ << ": " << frames_ << " frames, false obstacle pts/frame " << sum_false_obst_ / std::max<double>(1, frames_)
      << ", phantom cells " << (grid_frames_ ? sum_phantom_ / grid_frames_ : 0.0) << ", ground recall "
      << sum_ground_recall_ / std::max<double>(1, frames_) << ", dust recall (dusty frames) "
      << (dusty_frames_ ? sum_dust_recall_ / dusty_frames_ : 0.0);
    return o.str();
  }

  World world_;
  std::vector<Box> static_boxes_;
  std::unordered_map<int64_t, std::vector<int>> box_buckets_, cyl_buckets_;
  std::map<std::string, size_t> actor_index_;
  std::map<size_t, Eigen::Isometry3d> actor_pose_;
  Eigen::Isometry3d ego_ = Eigen::Isometry3d::Identity();
  bool have_ego_ = false, have_grid_ = false;
  nav_msgs::msg::OccupancyGrid grid_;
  std::string name_, log_path_;
  std::ofstream log_;
  double tol_, ego_front_, ego_rear_, ego_width_;
  std::array<std::array<long, 4>, kGtN> total_{};
  std::array<std::array<long, 40>, 2> hist_{};
  long frames_ = 0, grid_frames_ = 0, dusty_frames_ = 0, grid_div_ = 0, last_phantom_ = 0, last_occ_ = 0;
  double sum_false_obst_ = 0, sum_phantom_ = 0, sum_ground_recall_ = 0, sum_dust_recall_ = 0, sum_dust_prec_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cls_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<PerceptionEval>();
  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
