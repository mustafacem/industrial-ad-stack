// 3D world for the sensor-level simulator: height-field terrain with materials, oriented boxes,
// vertical cylinders, moving actors and volumetric dust. ROS-free.
#pragma once
#include <Eigen/Core>
#include <array>
#include <random>
#include <string>
#include <vector>

#include "iad_gz_sim/common/math.hpp"
#include "iad_gz_sim/common/point_types.hpp"
#include "iad_gz_sim/rng.hpp"

namespace iad::sim {

enum Material : uint8_t { kAsphalt = 0, kGravel, kDirt, kMud, kRock, kConcrete, kWater, kNumMaterials };

struct MaterialInfo {
  float reflectivity;  // Lambertian, 0..1 (drives LiDAR intensity)
  float slip;          // longitudinal slip ratio while driving
  float rr;            // rolling-resistance coefficient
  float cost;          // planning cost 0..99
  std::array<float, 3> color;
};
const MaterialInfo& materialInfo(uint8_t m);

using Color = std::array<float, 4>;

struct Terrain {
  int nx = 0, ny = 0;
  double res = 0.5, ox = 0, oy = 0;
  std::vector<float> h;
  std::vector<uint8_t> mat;
  std::vector<uint8_t> relief;  // 1 = not drivable: > 25 deg, or > 0.35 m off the local plane (ground truth)
  float hmin = 0, hmax = 0;

  void init(int nx_, int ny_, double res_, double ox_, double oy_);
  float& H(int i, int j) { return h[size_t(j) * size_t(nx) + size_t(i)]; }
  float H(int i, int j) const {
    i = std::clamp(i, 0, nx - 1), j = std::clamp(j, 0, ny - 1);
    return h[size_t(j) * size_t(nx) + size_t(i)];
  }
  uint8_t& M(int i, int j) { return mat[size_t(j) * size_t(nx) + size_t(i)]; }
  double xOf(int i) const { return ox + i * res; }  // heights are sampled at grid nodes
  double yOf(int j) const { return oy + j * res; }
  bool inside(double x, double y) const {
    return x >= ox && y >= oy && x <= ox + (nx - 1) * res && y <= oy + (ny - 1) * res;
  }
  double height(double x, double y) const;
  Eigen::Vector3d normal(double x, double y) const;
  uint8_t material(double x, double y) const;
  bool steep(double x, double y) const;
  void updateBounds();  // also recomputes the relief mask
  bool raycast(const Eigen::Vector3d& o, const Eigen::Vector3d& d, double tmax, double& t_hit) const;
};

struct Box {  // rotated about z only
  Eigen::Vector3d c = Eigen::Vector3d::Zero(), half = Eigen::Vector3d::Ones();
  double yaw = 0;
  float refl = 0.4f;
  Color color{0.6f, 0.6f, 0.6f, 1.f};
  uint8_t label = uint8_t(Label::kStatic);
  bool in_map = true;  // part of the prior site map (false = unmapped obstacle)
  bool visible = true;  // drawn in RViz (container stacks are drawn tier by tier instead)
  double radius() const { return half.head<2>().norm(); }
};

struct Cylinder {  // vertical axis
  double x = 0, y = 0, z0 = 0, z1 = 1, r = 0.5;
  float refl = 0.4f;
  Color color{0.5f, 0.5f, 0.5f, 1.f};
  uint8_t label = uint8_t(Label::kStatic);
  bool in_map = true;
  bool rock = false;  // drawn as a low-poly boulder instead of a cylinder
};

struct Actor {
  std::string kind;
  std::vector<Eigen::Vector2d> path;
  bool loop = true;           // false: ping-pong
  double speed = 3.0, s = 0.0, wait_at_end = 0.0, wait_left = 0.0;
  double start_delay = 0.0;
  int dir = 1;
  bool bidirectional = false;  // shuttles back and forth without turning (straddle carriers, AGVs)
  bool yields_to_traffic = false;  // gz: also give way to the other actors (not only to the ego)
  std::vector<Box> parts;     // actor frame: x forward, z up from ground
  bool emits_dust = false;
  Pose2 pose;
  double z = 0, v = 0, dust_timer = 0;
  bool init = false;
};

struct DustPuff {
  Eigen::Vector3d c = Eigen::Vector3d::Zero(), vel = Eigen::Vector3d::Zero();
  double r = 2.0, growth = 0.5, density = 0.15, age = 0, life = 12;
  double currentDensity() const { return density * std::max(0.0, 1.0 - age / life); }
};

struct DustSource {  // persistent emitter (dump face, crusher, haul-road wind erosion)
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  double rate = 1.0, radius = 6.0, density = 0.15, spread = 8.0;
  double timer = 0;
};

struct NamedPose {
  std::string name;
  double x = 0, y = 0, yaw = 0;
};

struct Route {
  std::string name;
  std::vector<Eigen::Vector2d> pts;
  double speed = 5.0;
  bool loop = false;
};

struct Hit {
  double t = 0;
  uint8_t label = 0;
  float refl = 0;
  Eigen::Vector3d n{0, 0, 1};
};

class World {
 public:
  std::string name;
  Terrain terrain;
  std::vector<Box> boxes;
  std::vector<Cylinder> cylinders;
  std::vector<Actor> actors;
  std::vector<DustPuff> dust;
  std::vector<DustSource> dust_sources;
  std::vector<Route> routes;
  std::vector<NamedPose> poses;
  std::vector<Box> visuals;  // drawn only, not ray-cast (container tiers, lane paint)
  Eigen::Vector2d wind{1.2, 0.4};
  double time = 0;
  size_t max_puffs = 160;

  void step(double dt, std::mt19937& rng);
  void emitDust(const Eigen::Vector3d& c, double r, double density, double life, const Eigen::Vector3d& vel);
  std::vector<Box> actorBoxes() const;
  const NamedPose* pose(const std::string& n) const;
  const Route* route(const std::string& n) const;
};

// Ray caster with a per-origin azimuth broad-phase: prepare() once per sensor pose, then cast
// thousands of rays from that origin.
class RayCaster {
 public:
  void prepare(const World& w, const std::vector<Box>& extra, const Eigen::Vector3d& origin, double range);
  bool castHard(const World& w, const Eigen::Vector3d& d, double tmax, Hit& hit) const;
  bool castDust(const World& w, const Eigen::Vector3d& d, double tmax, FastRng& rng, double& t_dust) const;

 private:
  static constexpr int kBins = 720;
  int binOf(double az) const;
  Eigen::Vector3d o_ = Eigen::Vector3d::Zero();
  std::vector<Box> boxes_;
  std::vector<Cylinder> cyls_;
  std::vector<std::vector<int>> box_bins_, cyl_bins_;
  std::vector<int> box_always_, cyl_always_;
  std::vector<int> dust_;
};

bool rayBox(const Box& b, const Eigen::Vector3d& o, const Eigen::Vector3d& d, double& t, Eigen::Vector3d& n);
bool rayCylinder(const Cylinder& c, const Eigen::Vector3d& o, const Eigen::Vector3d& d, double& t,
                 Eigen::Vector3d& n);

World makeQuarry(uint32_t seed);
World makePort(uint32_t seed);
World makeWorld(const std::string& name, uint32_t seed);

}  // namespace iad::sim
