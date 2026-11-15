// Reeds-Shepp curves: shortest paths for a car that drives forward and backward with a minimum turning
// radius (Reeds & Shepp 1990; formulas and the paper's typo fixes follow OMPL's ReedsSheppStateSpace).
// Used by Hybrid A* for analytic expansion ("shot to the goal") and as a non-holonomic heuristic.
#pragma once
#include <array>
#include <initializer_list>
#include <cmath>
#include <limits>
#include <vector>

namespace iad::planning {

namespace rs {
enum Seg { L, S, R, N };
inline const Seg kTypes[18][5] = {
    {L, R, L, N, N}, {R, L, R, N, N}, {L, R, L, R, N}, {R, L, R, L, N}, {L, R, S, L, N}, {R, L, S, R, N},
    {L, S, R, L, N}, {R, S, L, R, N}, {L, R, S, R, N}, {R, L, S, L, N}, {R, S, R, L, N}, {L, S, L, R, N},
    {L, S, R, N, N}, {R, S, L, N, N}, {L, S, L, N, N}, {R, S, R, N, N}, {L, R, S, L, R}, {R, L, S, R, L}};
constexpr double kPi = M_PI, kTwoPi = 2 * M_PI, kZero = 1e-9;

struct Path {
  const Seg* type = kTypes[0];
  std::array<double, 5> len{0, 0, 0, 0, 0};  // signed: negative = reverse (unit turning radius)
  double total = std::numeric_limits<double>::infinity();
  Path() = default;
  Path(const Seg* t, double a, double b, double c, double d = 0, double e = 0) : type(t), len{a, b, c, d, e} {
    total = std::abs(a) + std::abs(b) + std::abs(c) + std::abs(d) + std::abs(e);
  }
};

// Gear restriction for candidate words: 0 = any, +1 = forward only, -1 = reverse only.
inline thread_local int g_dir = 0;
inline bool gearOk(std::initializer_list<double> l) {
  if (g_dir == 0) return true;
  for (double v : l)
    if (v * g_dir < -1e-9) return false;
  return true;
}

inline double mod2pi(double x) {
  double v = std::fmod(x, kTwoPi);
  if (v < -kPi) v += kTwoPi;
  else if (v > kPi) v -= kTwoPi;
  return v;
}
inline void polar(double x, double y, double& r, double& th) { r = std::hypot(x, y), th = std::atan2(y, x); }
inline void tauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega) {
  const double delta = mod2pi(u - v), A = std::sin(u) - std::sin(delta), B = std::cos(u) - std::cos(delta) - 1.;
  const double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3;
  tau = (t2 < 0) ? mod2pi(t1 + kPi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}
inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
  polar(x - std::sin(phi), y - 1. + std::cos(phi), u, t);
  if (t >= -kZero) {
    v = mod2pi(phi - t);
    if (v >= -kZero) return true;
  }
  return false;
}
inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
  double t1, u1;
  polar(x + std::sin(phi), y - 1. - std::cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.) {
    u = std::sqrt(u1 - 4.);
    const double theta = std::atan2(2., u);
    t = mod2pi(t1 + theta), v = mod2pi(t - phi);
    return t >= -kZero && v >= -kZero;
  }
  return false;
}
inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x - std::sin(phi), eta = y - 1. + std::cos(phi);
  double u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.) {
    u = -2. * std::asin(.25 * u1);
    t = mod2pi(theta + .5 * u + kPi), v = mod2pi(phi - t + u);
    return t >= -kZero && u <= kZero;
  }
  return false;
}
inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi), rho = .25 * (2. + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.) {
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    return t >= -kZero && v <= kZero;
  }
  return false;
}
inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1) {
    u = -std::acos(rho);
    if (u >= -.5 * kPi) {
      tauOmega(u, u, xi, eta, phi, t, v);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}
inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x - std::sin(phi), eta = y - 1. + std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    const double r = std::sqrt(rho * rho - 4.);
    u = 2. - r, t = mod2pi(theta + std::atan2(r, -2.)), v = mod2pi(phi - .5 * kPi - t);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}
inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi);
  double rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.) {
    t = theta, u = 2. - rho, v = mod2pi(t + .5 * kPi - phi);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}
inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    u = 4. - std::sqrt(rho * rho - 4.);
    if (u <= kZero) {
      t = mod2pi(std::atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta)), v = mod2pi(t - phi);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

// Each family tries the base word and its timeflip / reflect / timeflip+reflect symmetries.
inline void CSC(double x, double y, double phi, Path& p) {
  double t, u, v, L, Lmin = p.total;
  auto take = [&](const Seg* ty, double a, double b, double c) {
    if (gearOk({a, b, c})) p = Path(ty, a, b, c), Lmin = L;
  };
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[14], t, u, v);
  if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[14], -t, -u, -v);
  if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[15], t, u, v);
  if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[15], -t, -u, -v);
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[12], t, u, v);
  if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[12], -t, -u, -v);
  if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[13], t, u, v);
  if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[13], -t, -u, -v);
}
inline void CCC(double x, double y, double phi, Path& p) {
  double t, u, v, L, Lmin = p.total;
  auto take = [&](const Seg* ty, double a, double b, double c) {
    if (gearOk({a, b, c})) p = Path(ty, a, b, c), Lmin = L;
  };
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[0], t, u, v);
  if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[0], -t, -u, -v);
  if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[1], t, u, v);
  if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[1], -t, -u, -v);
  const double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[0], v, u, t);
  if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[0], -v, -u, -t);
  if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[1], v, u, t);
  if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) take(kTypes[1], -v, -u, -t);
}
inline void CCCC(double x, double y, double phi, Path& p) {
  double t, u, v, L, Lmin = p.total;
  auto take = [&](const Seg* ty, double a, double b, double c, double d) {
    if (gearOk({a, b, c, d})) p = Path(ty, a, b, c, d), Lmin = L;
  };
  auto len = [&] { return std::abs(t) + 2. * std::abs(u) + std::abs(v); };
  if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[2], t, u, -u, v);
  if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[2], -t, -u, u, -v);
  if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[3], t, u, -u, v);
  if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[3], -t, -u, u, -v);
  if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[2], t, u, u, v);
  if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[2], -t, -u, -u, -v);
  if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[3], t, u, u, v);
  if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[3], -t, -u, -u, -v);
}
inline void CCSC(double x, double y, double phi, Path& p) {
  const double h = .5 * kPi;
  double t, u, v, L, Lmin = p.total - h;
  auto take = [&](const Seg* ty, double a, double b, double c, double d) {
    if (gearOk({a, b, c, d})) p = Path(ty, a, b, c, d), Lmin = L;
  };
  auto len = [&] { return std::abs(t) + std::abs(u) + std::abs(v); };
  if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[4], t, -h, u, v);
  if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[4], -t, h, -u, -v);
  if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[5], t, -h, u, v);
  if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[5], -t, h, -u, -v);
  if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[8], t, -h, u, v);
  if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[8], -t, h, -u, -v);
  if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[9], t, -h, u, v);
  if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[9], -t, h, -u, -v);
  const double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = len())) take(kTypes[6], v, u, -h, t);
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[6], -v, -u, h, -t);
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[7], v, u, -h, t);
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = len())) take(kTypes[7], -v, -u, h, -t);
  if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = len())) take(kTypes[10], v, u, -h, t);
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[10], -v, -u, h, -t);
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[11], v, u, -h, t);
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = len())) take(kTypes[11], -v, -u, h, -t);
}
inline void CCSCC(double x, double y, double phi, Path& p) {
  const double h = .5 * kPi;
  double t, u, v, L, Lmin = p.total - kPi;
  auto take = [&](const Seg* ty, double a, double b, double c, double d, double e) {
    if (gearOk({a, b, c, d, e})) p = Path(ty, a, b, c, d, e), Lmin = L;
  };
  auto len = [&] { return std::abs(t) + std::abs(u) + std::abs(v); };
  if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[16], t, -h, u, -h, v);
  if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[16], -t, h, -u, h, -v);
  if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = len())) take(kTypes[17], t, -h, u, -h, v);
  if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = len())) take(kTypes[17], -t, h, -u, h, -v);
}
}  // namespace rs

struct RsSample { double x, y, yaw; int dir; };

class ReedsShepp {
 public:
  explicit ReedsShepp(double radius) : r_(radius) {}

  // Shortest path in normalised coordinates between two poses; gear = +1/-1 restricts to one gear.
  rs::Path solve(double x0, double y0, double yaw0, double x1, double y1, double yaw1, int gear = 0) const {
    rs::g_dir = gear;
    const double dx = x1 - x0, dy = y1 - y0, c = std::cos(yaw0), s = std::sin(yaw0);
    const double x = (c * dx + s * dy) / r_, y = (-s * dx + c * dy) / r_, phi = yaw1 - yaw0;
    rs::Path p;
    rs::CSC(x, y, phi, p), rs::CCC(x, y, phi, p), rs::CCCC(x, y, phi, p), rs::CCSC(x, y, phi, p), rs::CCSCC(x, y, phi, p);
    rs::g_dir = 0;
    return p;
  }
  double distance(double x0, double y0, double yaw0, double x1, double y1, double yaw1) const {
    return r_ * solve(x0, y0, yaw0, x1, y1, yaw1).total;
  }

  // Samples every `step` metres (and at every segment end), starting after the start pose.
  std::vector<RsSample> sample(double x0, double y0, double yaw0, double x1, double y1, double yaw1, double step,
                               double* length = nullptr, int gear = 0) const {
    const rs::Path p = solve(x0, y0, yaw0, x1, y1, yaw1, gear);
    std::vector<RsSample> out;
    if (!std::isfinite(p.total)) return out;
    if (length) *length = r_ * p.total;
    double x = 0, y = 0, phi = 0;
    const double c0 = std::cos(yaw0), s0 = std::sin(yaw0);
    for (int i = 0; i < 5 && p.type[i] != rs::N; ++i) {
      const double seg = p.len[size_t(i)];
      if (std::abs(seg) < 1e-9) continue;
      const int n = std::max(1, int(std::ceil(std::abs(seg) * r_ / step)));
      const double d = seg / n;
      for (int k = 0; k < n; ++k) {
        if (p.type[i] == rs::L) x += std::sin(phi + d) - std::sin(phi), y += -std::cos(phi + d) + std::cos(phi), phi += d;
        else if (p.type[i] == rs::R) x += -std::sin(phi - d) + std::sin(phi), y += std::cos(phi - d) - std::cos(phi), phi -= d;
        else x += d * std::cos(phi), y += d * std::sin(phi);
        out.push_back({x0 + r_ * (c0 * x - s0 * y), y0 + r_ * (s0 * x + c0 * y), std::atan2(std::sin(yaw0 + phi), std::cos(yaw0 + phi)),
                       seg > 0 ? 1 : -1});
      }
    }
    return out;
  }

 private:
  double r_;
};

}  // namespace iad::planning
