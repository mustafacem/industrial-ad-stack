#include "iad_gz_sim/gnss_model.hpp"

#include <algorithm>

namespace iad::sim {

const char* gnssStatusName(GnssStatus s) {
  switch (s) {
    case GnssStatus::kFix: return "RTK_FIX";
    case GnssStatus::kFloat: return "RTK_FLOAT";
    case GnssStatus::kSingle: return "SINGLE";
    default: return "NO_FIX";
  }
}

// Fraction of the sky hemisphere blocked by structure (cranes, stacks, pit walls), elevation-weighted.
double GnssModel::skyBlocked(const World& w, const Eigen::Vector3d& antenna) {
  RayCaster rc;
  rc.prepare(w, {}, antenna, 300.0);
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

GnssSample GnssModel::sample(const World& w, const Eigen::Vector3d& antenna, double dt, FastRng& rng) {
  GnssSample s;
  s.sky_blocked = skyBlocked(w, antenna);
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
