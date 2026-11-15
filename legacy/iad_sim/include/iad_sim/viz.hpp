// RViz visuals for the simulated world: shaded TRIANGLE_LIST meshes (terrain tiles, static objects,
// actors, ego vehicle) and translucent dust particles.
#pragma once
#include <visualization_msgs/msg/marker_array.hpp>

#include "iad_common/vehicle.hpp"
#include "iad_sim/world.hpp"

namespace iad::sim {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class MeshBuilder {
 public:
  MeshBuilder(const std::string& frame, const std::string& ns, int id);
  void tri(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Color& col);
  void tri(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Color& ca,
           const Color& cb, const Color& cc);
  void quad(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d,
            const Color& col);
  void box(const Box& b);
  void cylinder(double x, double y, double z0, double z1, double r, const Color& col, int sides = 14);
  void wheel(const Eigen::Vector3d& c, double r, double width, const Color& tire, const Color& hub);
  void rock(double x, double y, double z0, double z1, double r, const Color& col, uint32_t seed);
  size_t triangles() const { return m_.points.size() / 3; }
  Marker& marker() { return m_; }

 private:
  Marker m_;
};

// Shading baked into vertex colours (RViz adds its own head-light on top).
Color shade(const Color& c, const Eigen::Vector3d& n, double ambient = 0.62);

// Tiles with strong relief (berms, cuts, pit benches) get the fine resolution, flat ones the coarse.
std::vector<Marker> terrainMarkers(const Terrain& t, double fine_res, double coarse_res, double tile,
                                   const std::string& frame);
std::vector<Marker> objectMarkers(const World& w, const std::string& frame);
std::vector<Marker> actorMarkers(const World& w);  // frame "actor_<i>", frame-locked
std::vector<Box> egoBodyBoxes(const VehicleParams& v);
std::vector<Marker> egoMarkers(const VehicleParams& v, const std::vector<Eigen::Isometry3d>& lidar_mounts,
                               const std::string& frame, const std::string& ns, float alpha = 1.f);
Marker dustMarker(const World& w, const std::string& frame, FastRng& rng);

}  // namespace iad::sim
