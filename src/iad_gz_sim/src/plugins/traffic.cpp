// iad_gz_sim::Traffic: scripted site traffic (haul trucks, light vehicles, straddle carriers, workers).
// Moves a kinematic, visual-only model along a polyline at a set speed: loop or ping-pong, optional wait
// at the ends, "bidirectional" shuttles that reverse without turning. It follows the terrain height and
// slope, smooths its heading at corners, and switches its dust emitter on while driving on loose ground.
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <sstream>
#include <vector>

#include "iad_gz_sim/terrain_grid.hpp"

namespace iad_gz_sim {

class Traffic : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    model_ = gz::sim::Model(entity);
    auto get = [&](const char* k, double d) { return sdf->HasElement(k) ? sdf->Get<double>(k) : d; };
    speed_ = get("speed", 3.0), wait_ = get("wait", 0.0), s_ = get("start_s", 0.0);
    loop_ = get("loop", 1) > 0.5, bidir_ = get("bidirectional", 0) > 0.5, dust_ = get("dust", 0) > 0.5;
    dir_ = get("start_dir", 1) < 0 ? -1 : 1;
    std::stringstream ss(sdf->Get<std::string>("waypoints"));
    double x, y;
    while (ss >> x >> y) pts_.push_back({x, y});
    for (size_t i = 0; i < pts_.size(); ++i) cum_.push_back(i ? cum_.back() + dist(pts_[i - 1], pts_[i]) : 0.0);
    if (sdf->HasElement("terrain_grid")) {
      const std::string p = iad::gz::findResource(sdf->Get<std::string>("terrain_grid"));
      if (p.empty() || !terrain_.load(p)) gzerr << "Traffic: cannot load terrain grid " << sdf->Get<std::string>("terrain_grid") << "\n";
    }
    name_ = model_.Name(ecm);
    pose_pub_ = node_.Advertise<gz::msgs::Pose_V>("/iad/traffic/poses");
    if (dust_) {
      const std::string n = model_.Name(ecm);
      dust_pub_ = node_.Advertise<gz::msgs::ParticleEmitter>("/model/" + n + "/link/link/particle_emitter/dust/cmd");
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || pts_.size() < 2) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    const double L = cum_.back();
    double v = 0;
    if (wait_left_ > 0) {
      wait_left_ -= dt;
    } else {
      s_ += dir_ * speed_ * dt, v = speed_;
      if (loop_) {
        s_ = std::fmod(s_ + L, L);
      } else if (s_ > L || s_ < 0) {
        s_ = std::clamp(s_, 0.0, L), dir_ = -dir_, wait_left_ = wait_;
      }
    }
    double px, py, yaw;
    at(s_, px, py, yaw);
    if (dir_ < 0 && !bidir_) yaw += M_PI;
    double dyaw = std::remainder(yaw - yaw_, 2 * M_PI);
    yaw_ = init_ ? yaw_ + dyaw * std::min(1.0, 3.0 * dt) : yaw;
    init_ = true;
    // Body attitude from the terrain under a 6 m x 3 m footprint.
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const double zf = terrain_.height(px + 3 * c, py + 3 * s), zr = terrain_.height(px - 3 * c, py - 3 * s);
    const double zl = terrain_.height(px - 1.5 * s, py + 1.5 * c), zrr = terrain_.height(px + 1.5 * s, py - 1.5 * c);
    const double pitch = -std::atan2(zf - zr, 6.0), roll = std::atan2(zl - zrr, 3.0);
    const double z = terrain_.height(px, py);
    model_.SetWorldPoseCmd(ecm, gz::math::Pose3d(px, py, z, roll, pitch, yaw_));
    // Named pose for evaluation (ros_gz_bridge -> tf2_msgs/TFMessage keeps header frame_id/child_frame_id).
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t - last_pub_ >= 0.05) {
      last_pub_ = t;
      gz::msgs::Pose_V v;
      auto* p = v.add_pose();
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      p->mutable_header()->mutable_stamp()->set_sec(ns / 1000000000), p->mutable_header()->mutable_stamp()->set_nsec(int(ns % 1000000000));
      auto* d1 = p->mutable_header()->add_data();
      d1->set_key("frame_id"), d1->add_value("map");
      auto* d2 = p->mutable_header()->add_data();
      d2->set_key("child_frame_id"), d2->add_value(name_);
      p->set_name(name_);
      const gz::math::Quaterniond q(roll, pitch, yaw_);
      p->mutable_position()->set_x(px), p->mutable_position()->set_y(py), p->mutable_position()->set_z(z);
      p->mutable_orientation()->set_w(q.W()), p->mutable_orientation()->set_x(q.X());
      p->mutable_orientation()->set_y(q.Y()), p->mutable_orientation()->set_z(q.Z());
      pose_pub_.Publish(v);
    }

    if (dust_) {
      const uint8_t m = terrain_.material(px, py);
      const bool on = v > 1.5 && (m == iad::gz::kDirt || m == iad::gz::kGravel || m == iad::gz::kMud);
      if (on != emitting_) {
        gz::msgs::ParticleEmitter msg;
        msg.mutable_emitting()->set_data(on);
        dust_pub_.Publish(msg);
        emitting_ = on;
      }
    }
  }

 private:
  static double dist(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    return std::hypot(b[0] - a[0], b[1] - a[1]);
  }
  void at(double s, double& x, double& y, double& yaw) const {
    size_t i = 1;
    while (i + 1 < pts_.size() && cum_[i] < s) ++i;
    const double seg = std::max(1e-6, cum_[i] - cum_[i - 1]), u = std::clamp((s - cum_[i - 1]) / seg, 0.0, 1.0);
    x = pts_[i - 1][0] + u * (pts_[i][0] - pts_[i - 1][0]);
    y = pts_[i - 1][1] + u * (pts_[i][1] - pts_[i - 1][1]);
    yaw = std::atan2(pts_[i][1] - pts_[i - 1][1], pts_[i][0] - pts_[i - 1][0]);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::vector<std::array<double, 2>> pts_;
  std::vector<double> cum_;
  double speed_ = 3, wait_ = 0, s_ = 0, wait_left_ = 0, yaw_ = 0;
  int dir_ = 1;
  bool loop_ = true, bidir_ = false, dust_ = false, init_ = false, emitting_ = false;
  iad::gz::TerrainGrid terrain_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher dust_pub_, pose_pub_;
  std::string name_;
  double last_pub_ = -1;
};

}  // namespace iad_gz_sim

GZ_ADD_PLUGIN(iad_gz_sim::Traffic, gz::sim::System, iad_gz_sim::Traffic::ISystemConfigure,
              iad_gz_sim::Traffic::ISystemPreUpdate)
