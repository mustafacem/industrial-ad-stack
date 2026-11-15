#include "iad_sim/sensors.hpp"

#include <omp.h>

namespace iad::sim {

Cloud scanLidar(const World& w, const std::vector<Box>& dynamic, const Eigen::Isometry3d& T_map_lidar,
                const LidarConfig& cfg, uint64_t seed, ScanStats* stats) {
  Stopwatch sw;
  RayCaster rc;
  rc.prepare(w, dynamic, T_map_lidar.translation(), cfg.range);
  const int n_az = int(std::lround(360.0 / cfg.hres)), C = cfg.channels;
  std::vector<double> va(static_cast<size_t>(C));
  for (int k = 0; k < C; ++k) va[size_t(k)] = deg2rad(cfg.vmin + (cfg.vmax - cfg.vmin) * k / std::max(1, C - 1));
  const Eigen::Matrix3d R = T_map_lidar.linear();
  std::vector<PointXYZIRL> buf(size_t(n_az) * size_t(C) * 2);
  std::vector<uint8_t> valid(buf.size(), 0);

#pragma omp parallel for schedule(dynamic, 32)
  for (int a = 0; a < n_az; ++a) {
    FastRng rng(seed * 1000003ull + uint64_t(a));
    const double az = 2.0 * kPi * a / n_az, ca = std::cos(az), sa = std::sin(az);
    for (int k = 0; k < C; ++k) {
      const double cv = std::cos(va[size_t(k)]), sv = std::sin(va[size_t(k)]);
      const Eigen::Vector3d dl(cv * ca, cv * sa, sv), d = R * dl;
      Hit hit;
      const bool hard = rc.castHard(w, d, cfg.range, hit);
      double td = 0;
      const bool dusty = rc.castDust(w, d, hard ? hit.t : cfg.range, rng, td);
      const size_t slot = (size_t(a) * size_t(C) + size_t(k)) * 2;
      const Eigen::Vector3d o = T_map_lidar.translation();
      auto emit = [&](size_t s, double t, double inten, uint8_t label, ReturnType rt) {
        if (t < cfg.min_range || t > cfg.range) return;
        const Eigen::Vector3d p = dl * t;
        if (label == uint8_t(Label::kStatic) || label == uint8_t(Label::kDynamic)) {
          const Eigen::Vector3d pw = o + d * t;
          if (pw.z() - w.terrain.height(pw.x(), pw.y()) < 0.3) label = uint8_t(Label::kObjectLow);
        }
        buf[s] = {float(p.x()), float(p.y()), float(p.z()), float(std::clamp(inten, 1.0, 255.0)), uint16_t(k),
                  uint8_t(rt), label};
        valid[s] = 1;
      };
      double inten = 0;
      if (hard) {
        const double cosi = std::abs(hit.n.dot(d));
        inten = hit.refl > 0.85f ? 255.0 : 255.0 * hit.refl * (0.3 + 0.7 * cosi) * (1.0 + 0.08 * rng.normal());
      }
      const double t_hard = hard ? hit.t + cfg.noise * rng.normal() : 0.0;
      if (dusty) {
        // Dust backscatter: weak, diffuse first echo. Part of the pulse still reaches the target.
        const double di = 2.0 + 10.0 * rng.uniform();
        const bool pass = hard && rng.uniform() < 0.7;
        if (cfg.dual) {
          emit(slot, td + 0.05 * rng.normal(), di, uint8_t(Label::kDust), pass ? ReturnType::kFirst : ReturnType::kSingle);
          if (pass) emit(slot + 1, t_hard, 0.7 * inten, hit.label, ReturnType::kLast);
        } else if (pass && rng.uniform() < 0.5) {
          emit(slot, t_hard, 0.7 * inten, hit.label, ReturnType::kSingle);
        } else {
          emit(slot, td + 0.05 * rng.normal(), di, uint8_t(Label::kDust), ReturnType::kSingle);
        }
      } else if (hard) {
        emit(slot, t_hard, inten, hit.label, ReturnType::kSingle);
      }
    }
  }
  Cloud out;
  out.reserve(buf.size() / 2);
  size_t n_dust = 0;
  for (size_t i = 0; i < buf.size(); ++i)
    if (valid[i]) out.push_back(buf[i]), n_dust += buf[i].label == uint8_t(Label::kDust);
  if (stats) stats->ms = sw.ms(), stats->points = out.size(), stats->dust = n_dust;
  return out;
}

const char* gnssStatusName(GnssStatus s) {
  switch (s) {
    case GnssStatus::kFix: return "RTK_FIX";
    case GnssStatus::kFloat: return "RTK_FLOAT";
    case GnssStatus::kSingle: return "SINGLE";
    default: return "NO_FIX";
  }
}

// Fraction of the sky hemisphere blocked by structure (cranes, stacks, pit walls), elevation-weighted.
double GnssModel::skyBlocked(const World& w, const std::vector<Box>& dynamic, const Eigen::Vector3d& antenna) {
  RayCaster rc;
  rc.prepare(w, dynamic, antenna, 300.0);
  static const double kElev[] = {12, 25, 40, 58, 78};
  static const double kW[] = {0.5, 1.0, 1.0, 1.0, 1.0};
  double blocked = 0, total = 0;
  for (int e = 0; e < 5; ++e)
    for (int a = 0; a < 18; ++a) {
      const double el = deg2rad(kElev[e]), az = 2 * kPi * a / 18.0;
      const Eigen::Vector3d d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      Hit h;
      blocked += kW[e] * rc.castHard(w, d, 300.0, h), total += kW[e];
    }
  Hit h;
  const double zen = rc.castHard(w, Eigen::Vector3d(0, 0, 1), 300.0, h) ? 1.0 : 0.0;
  return std::min(1.0, blocked / total + 0.35 * zen);
}

GnssSample GnssModel::sample(const World& w, const std::vector<Box>& dynamic, const Eigen::Vector3d& antenna,
                             double dt, FastRng& rng) {
  GnssSample s;
  s.sky_blocked = skyBlocked(w, dynamic, antenna);
  const GnssStatus target = s.sky_blocked < 0.06   ? GnssStatus::kFix
                            : s.sky_blocked < 0.16 ? GnssStatus::kFloat
                            : s.sky_blocked < 0.40 ? GnssStatus::kSingle
                                                   : GnssStatus::kNoFix;
  // Degrades at once; RTK needs a few seconds of clear sky to re-converge, one level at a time.
  if (int(target) < int(status_)) {
    status_ = target, upgrade_timer_ = 0;
  } else if (int(target) > int(status_)) {
    upgrade_timer_ += dt;
    if (upgrade_timer_ > (status_ == GnssStatus::kFloat ? 4.0 : 2.0)) status_ = GnssStatus(int(status_) + 1), upgrade_timer_ = 0;
  }
  s.status = status_;
  // Multipath bias: first-order Gauss-Markov whose spread depends on the fix type.
  const double tau = 8.0;
  // Reflections off steel walls grow with how much of the sky they hide.
  const double sb = status_ == GnssStatus::kFix ? 0.01 : status_ == GnssStatus::kFloat ? 0.35 + 2.0 * s.sky_blocked : 1.5 + 6.0 * s.sky_blocked;
  bias_ += -bias_ * (dt / tau) + sb * std::sqrt(2 * dt / tau) * Eigen::Vector2d(rng.normal(), rng.normal());
  if (status_ == GnssStatus::kSingle && rng.uniform() < 0.03) bias_ += 3.0 * Eigen::Vector2d(rng.normal(), rng.normal());
  if (status_ == GnssStatus::kFix) bias_ *= 0.5;
  const double white = status_ == GnssStatus::kFix ? 0.012 : status_ == GnssStatus::kFloat ? 0.10 : 0.7;
  s.sigma_reported = status_ == GnssStatus::kFix ? 0.02 : status_ == GnssStatus::kFloat ? 0.25 : 1.2;  // optimistic
  if (status_ == GnssStatus::kNoFix) return s;
  const Eigen::Vector2d e = bias_ + white * Eigen::Vector2d(rng.normal(), rng.normal());
  s.valid = true;
  s.pos = antenna + Eigen::Vector3d(e.x(), e.y(), 1.5 * white * rng.normal());
  s.error = e.norm();
  return s;
}

}  // namespace iad::sim
