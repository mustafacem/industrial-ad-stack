#include "iad_gz_sim/world.hpp"

#include <cmath>
#include <limits>

#include "iad_gz_sim/rng.hpp"

namespace iad::sim {

const MaterialInfo& materialInfo(uint8_t m) {
  //                               refl   slip  rr     cost  color
  static const MaterialInfo kTable[kNumMaterials] = {
      {0.10f, 0.00f, 0.015f, 0.f, {0.27f, 0.28f, 0.30f}},   // asphalt
      {0.22f, 0.03f, 0.030f, 4.f, {0.66f, 0.63f, 0.58f}},   // gravel
      {0.25f, 0.06f, 0.050f, 15.f, {0.63f, 0.51f, 0.39f}},  // dirt
      {0.07f, 0.20f, 0.100f, 45.f, {0.30f, 0.23f, 0.17f}},  // mud
      {0.35f, 0.02f, 0.030f, 30.f, {0.55f, 0.50f, 0.45f}},  // rock
      {0.30f, 0.00f, 0.012f, 0.f, {0.60f, 0.60f, 0.58f}},   // concrete
      {0.00f, 1.00f, 1.000f, 100.f, {0.09f, 0.20f, 0.30f}}, // water (absorbs the laser)
  };
  return kTable[m < kNumMaterials ? m : uint8_t(kDirt)];
}

// ---------------------------------------------------------------- terrain
void Terrain::init(int nx_, int ny_, double res_, double ox_, double oy_) {
  nx = nx_, ny = ny_, res = res_, ox = ox_, oy = oy_;
  h.assign(size_t(nx) * size_t(ny), 0.f);
  mat.assign(size_t(nx) * size_t(ny), kDirt);
}

void Terrain::updateBounds() {
  hmin = std::numeric_limits<float>::max(), hmax = std::numeric_limits<float>::lowest();
  for (float v : h) hmin = std::min(hmin, v), hmax = std::max(hmax, v);
  // Ground-truth traversability per node: slope over +-1.5 m > 25 deg (flanks, faces, cut walls), or the
  // node stands > 0.4 m above the lowest ground within 1.5 m (berm crests, mounds). Road next to a berm
  // stays drivable.
  // NB: this is a non-const member, so H(i, j) would pick the unclamped (reference) overload: use an
  // explicitly clamped read, or the border windows read outside the height buffer.
  const int r = std::max(1, int(std::lround(1.5 / res)));
  auto Hc = [&](int i, int j) {
    return h[size_t(std::clamp(j, 0, ny - 1)) * size_t(nx) + size_t(std::clamp(i, 0, nx - 1))];
  };
  relief.assign(h.size(), 0);
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      const double gx = (Hc(i + r, j) - Hc(i - r, j)) / (2.0 * r * res), gy = (Hc(i, j + r) - Hc(i, j - r)) / (2.0 * r * res);
      bool steep = std::hypot(gx, gy) > 0.466;
      float lo = Hc(i, j);
      for (int b = -r; b <= r && !steep; b += 1)
        for (int a = -r; a <= r; a += 1) lo = std::min(lo, Hc(i + a, j + b));
      relief[size_t(j) * nx + i] = steep || Hc(i, j) - lo > 0.4f;
    }
}

bool Terrain::steep(double x, double y) const {
  const int i = std::clamp(int(std::lround((x - ox) / res)), 0, nx - 1);
  const int j = std::clamp(int(std::lround((y - oy) / res)), 0, ny - 1);
  return !relief.empty() && relief[size_t(j) * size_t(nx) + size_t(i)];
}

double Terrain::height(double x, double y) const {
  double fx = std::clamp((x - ox) / res, 0.0, nx - 1.0001), fy = std::clamp((y - oy) / res, 0.0, ny - 1.0001);
  const int i = int(fx), j = int(fy);
  const double ax = fx - i, ay = fy - j;
  return (1 - ax) * (1 - ay) * H(i, j) + ax * (1 - ay) * H(i + 1, j) + (1 - ax) * ay * H(i, j + 1) +
         ax * ay * H(i + 1, j + 1);
}

Eigen::Vector3d Terrain::normal(double x, double y) const {
  const double e = res;
  const double dzdx = (height(x + e, y) - height(x - e, y)) / (2 * e);
  const double dzdy = (height(x, y + e) - height(x, y - e)) / (2 * e);
  return Eigen::Vector3d(-dzdx, -dzdy, 1.0).normalized();
}

uint8_t Terrain::material(double x, double y) const {
  const int i = std::clamp(int(std::lround((x - ox) / res)), 0, nx - 1);
  const int j = std::clamp(int(std::lround((y - oy) / res)), 0, ny - 1);
  return mat[size_t(j) * size_t(nx) + size_t(i)];
}

// Height-field ray march: the step grows with the vertical gap (safe for slopes up to ~60 deg),
// then the first sign change is refined by bisection.
bool Terrain::raycast(const Eigen::Vector3d& o, const Eigen::Vector3d& d, double tmax, double& t_hit) const {
  const double min_step = 0.6 * res;
  double t = 0.0;
  double gap = o.z() - height(o.x(), o.y());
  if (gap < 0) return false;
  while (t < tmax) {
    const double z = o.z() + t * d.z();
    if (d.z() >= 0 && z > hmax + 0.05) return false;
    const double tn = std::min(t + std::max(min_step, 0.5 * gap), tmax);
    const Eigen::Vector3d p = o + tn * d;
    if (!inside(p.x(), p.y())) return false;
    const double g = p.z() - height(p.x(), p.y());
    if (g <= 0) {
      double a = t, b = tn;
      for (int k = 0; k < 10; ++k) {
        const double m = 0.5 * (a + b);
        const Eigen::Vector3d q = o + m * d;
        (q.z() - height(q.x(), q.y()) > 0 ? a : b) = m;
      }
      t_hit = 0.5 * (a + b);
      return true;
    }
    t = tn, gap = g;
  }
  return false;
}

// ---------------------------------------------------------------- primitives
bool rayBox(const Box& b, const Eigen::Vector3d& o, const Eigen::Vector3d& d, double& t, Eigen::Vector3d& n) {
  const double c = std::cos(b.yaw), s = std::sin(b.yaw);
  const Eigen::Vector3d po = o - b.c;
  const double lo[3] = {c * po.x() + s * po.y(), -s * po.x() + c * po.y(), po.z()};
  const double ld[3] = {c * d.x() + s * d.y(), -s * d.x() + c * d.y(), d.z()};
  double tn = -1e30, tf = 1e30, sign = 0;
  int axis = -1;
  for (int k = 0; k < 3; ++k) {
    if (std::abs(ld[k]) < 1e-12) {
      if (std::abs(lo[k]) > b.half[k]) return false;
      continue;
    }
    const double inv = 1.0 / ld[k];
    double t1 = (-b.half[k] - lo[k]) * inv, t2 = (b.half[k] - lo[k]) * inv, sg = -1;
    if (t1 > t2) std::swap(t1, t2), sg = 1;
    if (t1 > tn) tn = t1, axis = k, sign = sg;
    tf = std::min(tf, t2);
    if (tn > tf) return false;
  }
  if (axis < 0 || tn < 1e-6) return false;  // behind the origin, or the origin is inside
  t = tn;
  Eigen::Vector3d nl = Eigen::Vector3d::Zero();
  nl[axis] = sign;
  n = Eigen::Vector3d(c * nl.x() - s * nl.y(), s * nl.x() + c * nl.y(), nl.z());
  return true;
}

bool rayCylinder(const Cylinder& cy, const Eigen::Vector3d& o, const Eigen::Vector3d& d, double& t,
                 Eigen::Vector3d& n) {
  double best = 1e30;
  bool hit = false;
  const double px = o.x() - cy.x, py = o.y() - cy.y;
  const double a = d.x() * d.x() + d.y() * d.y();
  if (a > 1e-12) {
    const double b = 2 * (px * d.x() + py * d.y()), c = px * px + py * py - cy.r * cy.r;
    const double disc = b * b - 4 * a * c;
    if (disc >= 0 && c > 0) {
      const double t0 = (-b - std::sqrt(disc)) / (2 * a);
      const double z = o.z() + t0 * d.z();
      if (t0 > 1e-6 && z >= cy.z0 && z <= cy.z1) {
        best = t0, hit = true;
        n = Eigen::Vector3d(px + t0 * d.x(), py + t0 * d.y(), 0).normalized();
      }
    }
  }
  if (d.z() < -1e-9) {  // top cap
    const double tc = (cy.z1 - o.z()) / d.z();
    const double qx = px + tc * d.x(), qy = py + tc * d.y();
    if (tc > 1e-6 && tc < best && qx * qx + qy * qy <= cy.r * cy.r) best = tc, hit = true, n = {0, 0, 1};
  }
  if (hit) t = best;
  return hit;
}

// ---------------------------------------------------------------- ray caster
int RayCaster::binOf(double az) const {
  int b = int(std::floor((az + kPi) / (2 * kPi) * kBins));
  return ((b % kBins) + kBins) % kBins;
}

void RayCaster::prepare(const World& w, const std::vector<Box>& extra, const Eigen::Vector3d& origin,
                        double range) {
  o_ = origin;
  boxes_.clear(), cyls_.clear(), box_always_.clear(), cyl_always_.clear(), dust_.clear();
  box_bins_.assign(kBins, {});
  cyl_bins_.assign(kBins, {});
  const double margin = deg2rad(0.6);
  auto addSpan = [&](std::vector<std::vector<int>>& bins, int idx, double a0, double a1) {
    const int b0 = binOf(a0), n = std::min(kBins, int(std::ceil((a1 - a0) / (2 * kPi) * kBins)) + 1);
    for (int k = 0; k <= n; ++k) bins[(b0 + k) % kBins].push_back(idx);
  };
  auto addBox = [&](const Box& b) {
    const double dx = b.c.x() - o_.x(), dy = b.c.y() - o_.y(), dist = std::hypot(dx, dy), rad = b.radius();
    if (dist - rad > range) return;
    const int idx = int(boxes_.size());
    boxes_.push_back(b);
    if (dist < rad + 0.5) return box_always_.push_back(idx);
    const double ac = std::atan2(dy, dx), c = std::cos(b.yaw), s = std::sin(b.yaw);
    double lo = 1e9, hi = -1e9;
    for (int k = 0; k < 4; ++k) {
      const double lx = (k & 1 ? 1 : -1) * b.half.x(), ly = (k & 2 ? 1 : -1) * b.half.y();
      const double da = wrapAngle(std::atan2(dy + s * lx + c * ly, dx + c * lx - s * ly) - ac);
      lo = std::min(lo, da), hi = std::max(hi, da);
    }
    addSpan(box_bins_, idx, ac + lo - margin, ac + hi + margin);
  };
  for (const auto& b : w.boxes) addBox(b);
  for (const auto& b : extra) addBox(b);
  for (const auto& c : w.cylinders) {
    const double dx = c.x - o_.x(), dy = c.y - o_.y(), dist = std::hypot(dx, dy);
    if (dist - c.r > range) continue;
    const int idx = int(cyls_.size());
    cyls_.push_back(c);
    if (dist < c.r + 0.5) {
      cyl_always_.push_back(idx);
      continue;
    }
    const double ac = std::atan2(dy, dx), half = std::asin(std::min(1.0, c.r / dist)) + margin;
    addSpan(cyl_bins_, idx, ac - half, ac + half);
  }
  for (size_t k = 0; k < w.dust.size(); ++k)
    if ((w.dust[k].c - o_).norm() - w.dust[k].r < range && w.dust[k].currentDensity() > 1e-4)
      dust_.push_back(int(k));
}

bool RayCaster::castHard(const World& w, const Eigen::Vector3d& d, double tmax, Hit& hit) const {
  double best = tmax, t;
  bool found = false;
  Eigen::Vector3d n;
  const int bin = binOf(std::atan2(d.y(), d.x()));
  auto testBox = [&](int i) {
    if (rayBox(boxes_[i], o_, d, t, n) && t < best)
      best = t, hit.n = n, hit.label = boxes_[i].label, hit.refl = boxes_[i].refl, found = true;
  };
  auto testCyl = [&](int i) {
    if (rayCylinder(cyls_[i], o_, d, t, n) && t < best)
      best = t, hit.n = n, hit.label = cyls_[i].label, hit.refl = cyls_[i].refl, found = true;
  };
  for (int i : box_bins_[bin]) testBox(i);
  for (int i : box_always_) testBox(i);
  for (int i : cyl_bins_[bin]) testCyl(i);
  for (int i : cyl_always_) testCyl(i);
  if (w.terrain.raycast(o_, d, best, t)) {
    const Eigen::Vector3d p = o_ + t * d;
    const uint8_t m = w.terrain.material(p.x(), p.y());
    if (m == kWater) return false;  // specular water: no return
    best = t, found = true;
    hit.refl = materialInfo(m).reflectivity, hit.n = w.terrain.normal(p.x(), p.y());
    hit.label = uint8_t(w.terrain.steep(p.x(), p.y()) ? Label::kSteep : Label::kGround);
  }
  hit.t = best;
  return found;
}

// Each puff is an inhomogeneous Poisson process of backscatter events along the beam with a
// parabolic density profile; sample the first event by thinning. Puffs are independent, so the
// first echo overall is the minimum over puffs.
bool RayCaster::castDust(const World& w, const Eigen::Vector3d& d, double tmax, FastRng& rng,
                         double& t_dust) const {
  double best = tmax;
  bool found = false;
  for (int k : dust_) {
    const DustPuff& p = w.dust[size_t(k)];
    const double sig = p.currentDensity();
    const Eigen::Vector3d oc = o_ - p.c;
    const double b = oc.dot(d), c = oc.squaredNorm() - p.r * p.r, disc = b * b - c;
    if (disc <= 0) continue;
    const double sq = std::sqrt(disc), t0 = std::max(0.0, -b - sq), t1 = std::min(best, -b + sq);
    if (t0 >= t1) continue;
    for (double t = t0;;) {
      t += -std::log(rng.uniform() + 1e-12) / sig;
      if (t >= t1) break;
      const double rr = ((o_ + t * d) - p.c).squaredNorm() / (p.r * p.r);
      if (rng.uniform() < 1.0 - rr) {
        best = t, found = true;
        break;
      }
    }
  }
  if (found) t_dust = best;
  return found;
}

// ---------------------------------------------------------------- world dynamics
static void actorPose(const Actor& a, double s, Pose2& out) {
  const auto& P = a.path;
  double acc = 0;
  for (size_t i = 0; i + 1 < P.size(); ++i) {
    const double L = (P[i + 1] - P[i]).norm();
    if (s <= acc + L || i + 2 == P.size()) {
      const double u = L > 1e-9 ? std::clamp((s - acc) / L, 0.0, 1.0) : 0.0;
      const Eigen::Vector2d q = P[i] + u * (P[i + 1] - P[i]);
      out.x = q.x(), out.y = q.y();
      out.yaw = std::atan2(P[i + 1].y() - P[i].y(), P[i + 1].x() - P[i].x());
      return;
    }
    acc += L;
  }
  out.x = P.front().x(), out.y = P.front().y();
}

static double pathLength(const std::vector<Eigen::Vector2d>& P) {
  double L = 0;
  for (size_t i = 0; i + 1 < P.size(); ++i) L += (P[i + 1] - P[i]).norm();
  return L;
}

void World::emitDust(const Eigen::Vector3d& c, double r, double density, double life, const Eigen::Vector3d& vel) {
  DustPuff p;
  p.c = c, p.r = r, p.density = density, p.life = life, p.vel = vel;
  p.growth = 0.45;
  dust.push_back(p);
  if (dust.size() > max_puffs) dust.erase(dust.begin());
}

void World::step(double dt, std::mt19937& rng) {
  time += dt;
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  for (auto& a : actors) {
    if (a.path.size() < 2) continue;
    const double L = pathLength(a.path);
    if (time < a.start_delay) {
      a.v = 0;
    } else if (a.wait_left > 0) {
      a.wait_left -= dt, a.v = 0;
    } else {
      a.s += a.dir * a.speed * dt, a.v = a.speed;
      if (a.loop) {
        a.s = std::fmod(a.s + L, L);
      } else if (a.s > L || a.s < 0) {
        a.s = std::clamp(a.s, 0.0, L), a.dir = -a.dir, a.wait_left = a.wait_at_end;
      }
    }
    Pose2 p;
    actorPose(a, a.s, p);
    if (a.dir < 0 && !a.bidirectional) p.yaw = wrapAngle(p.yaw + kPi);
    // Smooth heading so vehicles don't snap at polyline corners.
    const double dyaw = wrapAngle(p.yaw - a.pose.yaw);
    a.pose.x = p.x, a.pose.y = p.y;
    a.pose.yaw = a.init ? wrapAngle(a.pose.yaw + dyaw * std::min(1.0, 3.0 * dt)) : p.yaw;
    a.init = true;
    a.z = terrain.height(p.x, p.y);
    if (a.emits_dust && a.v > 1.5) {
      const uint8_t m = terrain.material(p.x, p.y);
      if (m == kDirt || m == kGravel || m == kMud) {
        a.dust_timer += dt;
        if (a.dust_timer > 0.3) {
          a.dust_timer = 1e-6;
          const Eigen::Vector3d c(p.x - 4.0 * std::cos(a.pose.yaw) + U(rng), p.y - 4.0 * std::sin(a.pose.yaw) + U(rng),
                                  a.z + 1.4);
          const Eigen::Vector3d vel(wind.x() + 0.2 * a.v * std::cos(a.pose.yaw), wind.y() + 0.2 * a.v * std::sin(a.pose.yaw), 0.15);
          emitDust(c, 2.0, 0.30, 11.0, vel);
        }
      }
    }
  }
  for (auto& s : dust_sources) {
    s.timer += dt;
    if (s.timer > 1.0 / s.rate) {
      s.timer = 0;
      const Eigen::Vector3d c = s.c + Eigen::Vector3d(U(rng) * s.spread, U(rng) * s.spread, 0.5 * U(rng));
      emitDust(c, s.radius, s.density, 14.0, Eigen::Vector3d(wind.x(), wind.y(), 0.1));
    }
  }
  for (auto& p : dust) {
    p.age += dt;
    p.r += p.growth * dt;
    p.c += p.vel * dt;
    const Eigen::Vector3d target(wind.x(), wind.y(), 0.08);
    p.vel += (target - p.vel) * std::min(1.0, dt / 3.0);
    const double floor_z = terrain.height(p.c.x(), p.c.y()) + 0.5 * p.r;
    if (p.c.z() < floor_z) p.c.z() = floor_z;
  }
  dust.erase(std::remove_if(dust.begin(), dust.end(), [](const DustPuff& p) { return p.age >= p.life; }), dust.end());
}

std::vector<Box> World::actorBoxes() const {
  std::vector<Box> out;
  for (const auto& a : actors) {
    const double c = std::cos(a.pose.yaw), s = std::sin(a.pose.yaw);
    for (Box b : a.parts) {
      const Eigen::Vector3d l = b.c;
      b.c = Eigen::Vector3d(a.pose.x + c * l.x() - s * l.y(), a.pose.y + s * l.x() + c * l.y(), a.z + l.z());
      b.yaw = wrapAngle(a.pose.yaw + b.yaw);
      b.label = uint8_t(Label::kDynamic);
      b.in_map = false;
      out.push_back(b);
    }
  }
  return out;
}

const NamedPose* World::pose(const std::string& n) const {
  for (const auto& p : poses)
    if (p.name == n) return &p;
  return nullptr;
}

const Route* World::route(const std::string& n) const {
  for (const auto& r : routes)
    if (r.name == n) return &r;
  return nullptr;
}

World makeWorld(const std::string& name, uint32_t seed) {
  return name == "port" ? makePort(seed) : makeQuarry(seed);
}

}  // namespace iad::sim
