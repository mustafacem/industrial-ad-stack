// WGS84 geodetic <-> local ENU (exact, via ECEF), the same model Gazebo's navsat sensor uses.
#pragma once
#include <Eigen/Core>
#include <cmath>

namespace iad::geo {

struct Origin {
  double lat0 = 0, lon0 = 0, alt0 = 0;
  Eigen::Vector3d ecef0 = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // ECEF -> ENU
  Origin() = default;
  Origin(double lat, double lon, double alt) : lat0(lat), lon0(lon), alt0(alt) {
    ecef0 = ecef(lat, lon, alt);
    const double la = lat * M_PI / 180, lo = lon * M_PI / 180;
    R << -std::sin(lo), std::cos(lo), 0, -std::sin(la) * std::cos(lo), -std::sin(la) * std::sin(lo), std::cos(la),
        std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la);
  }
  static Eigen::Vector3d ecef(double lat, double lon, double alt) {
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    const double la = lat * M_PI / 180, lo = lon * M_PI / 180;
    const double N = a / std::sqrt(1 - e2 * std::sin(la) * std::sin(la));
    return {(N + alt) * std::cos(la) * std::cos(lo), (N + alt) * std::cos(la) * std::sin(lo), (N * (1 - e2) + alt) * std::sin(la)};
  }
  Eigen::Vector3d toEnu(double lat, double lon, double alt) const { return R * (ecef(lat, lon, alt) - ecef0); }
  void toGeodetic(const Eigen::Vector3d& enu, double& lat, double& lon, double& alt) const {
    const Eigen::Vector3d p = ecef0 + R.transpose() * enu;
    const double a = 6378137.0, e2 = 6.69437999014e-3, b = a * std::sqrt(1 - e2), ep2 = (a * a - b * b) / (b * b);
    const double r = std::hypot(p.x(), p.y()), th = std::atan2(a * p.z(), b * r);
    const double la = std::atan2(p.z() + ep2 * b * std::pow(std::sin(th), 3), r - e2 * a * std::pow(std::cos(th), 3));
    const double N = a / std::sqrt(1 - e2 * std::sin(la) * std::sin(la));
    lat = la * 180 / M_PI, lon = std::atan2(p.y(), p.x()) * 180 / M_PI, alt = r / std::cos(la) - N;
  }
};

}  // namespace iad::geo
