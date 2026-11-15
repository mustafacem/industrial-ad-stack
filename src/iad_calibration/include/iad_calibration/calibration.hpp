// Target-less multi-LiDAR extrinsic calibration (ROS-free), in three stages:
//  1. Ground plane (vehicle at standstill on flat ground): the plane fitted in the sensor frame gives
//     the base z axis in sensor coordinates -> roll, pitch, and the mounting height.
//  2. Mapping-based registration (vehicle driving): a reference map is built from the reference LiDAR
//     (roof) with the vehicle trajectory; every keyframe of a corner LiDAR is registered jointly to it,
//     solving the remaining yaw, x, y of its extrinsic (Gauss-Newton, point-to-plane on map voxels).
//  3. Joint six-DOF refinement against the same map removes residual ground-plane errors.
#pragma once
#include <Eigen/Dense>
#include <random>
#include <vector>

#include "iad_calibration/common.hpp"

namespace iad::calibration {

struct Plane {
  Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
  double d = 0;  // n.p + d = 0, n oriented "up" (towards the sensor)
  int inliers = 0;
};

// RANSAC on (roughly) ground points, then a PCA refit on the inliers.
inline Plane fitGroundPlane(const std::vector<Eigen::Vector3d>& pts, double th = 0.08, int iters = 300, uint32_t seed = 3) {
  Plane best;
  if (pts.size() < 10) return best;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<size_t> U(0, pts.size() - 1);
  for (int it = 0; it < iters; ++it) {
    const Eigen::Vector3d a = pts[U(rng)], b = pts[U(rng)], c = pts[U(rng)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() < 1e-6) continue;
    n.normalize();
    if (n.z() < 0) n = -n;
    const double d = -n.dot(a);
    if (d < 0.3) continue;  // the sensor (origin) must be above the plane
    int in = 0;
    for (size_t k = 0; k < pts.size(); k += 3) in += std::abs(n.dot(pts[k]) + d) < th;
    if (in > best.inliers) best.n = n, best.d = d, best.inliers = in;
  }
  std::vector<Eigen::Vector3d> in;
  for (const auto& p : pts)
    if (std::abs(best.n.dot(p) + best.d) < th) in.push_back(p);
  if (in.size() < 10) return best;
  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  for (const auto& p : in) mu += p;
  mu /= double(in.size());
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (const auto& p : in) C += (p - mu) * (p - mu).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  Eigen::Vector3d n = es.eigenvectors().col(0);
  if (n.dot(best.n) < 0) n = -n;
  best.n = n, best.d = -n.dot(mu), best.inliers = int(in.size());
  return best;
}

// Replace roll, pitch and z of T_base_lidar so that the fitted ground plane becomes base z = 0.
// (n in the sensor frame = R^T e_z = (-sin p, sin r cos p, cos r cos p) for R = Rz(yaw) Ry(p) Rx(r).)
inline Eigen::Isometry3d applyGroundPlane(const Eigen::Isometry3d& T, const Plane& g) {
  const Eigen::Vector3d rpy = rpyFromQuat(Eigen::Quaterniond(T.linear()));
  const double pitch = -std::asin(std::clamp(g.n.x(), -1.0, 1.0)), roll = std::atan2(g.n.y(), g.n.z());
  Eigen::Isometry3d out = T;
  out.linear() = quatFromRPY(roll, pitch, rpy.z()).toRotationMatrix();
  out.translation().z() = g.d;
  return out;
}

struct Keyframe {
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> pts;  // corner-LiDAR points in its own frame (downsampled)
};

struct StepResult {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  double rms = 0;  // RMS point-to-plane residual of matched points [m]
  int matched = 0;
  double step = 0;
};

// One Gauss-Newton step on (yaw, x, y) of T_base_lidar, all keyframes jointly. Perturbation is applied
// in the base frame about the sensor position: R <- Rz(dyaw) R, t <- t + (dx, dy, 0).
inline StepResult refineStep(const SurfelMap& map, const std::vector<Keyframe>& kfs, const Eigen::Isometry3d& T,
                             double score_scale = 0.25) {
  StepResult r;
  r.T = T;
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  double sse = 0;
  static const int kNb[7][3] = {{0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const auto& kf : kfs) {
    const Eigen::Matrix3d Rmb = kf.T_map_base.linear();
    for (const auto& p : kf.pts) {
      const Eigen::Vector3d pb = T * p, rel = pb - T.translation();
      const Eigen::Vector3d q = kf.T_map_base * pb;
      // d q / d (yaw, x, y) in the map frame
      Eigen::Matrix<double, 3, 3> J;
      J.col(0) = Rmb * Eigen::Vector3d(-rel.y(), rel.x(), 0.0);
      J.col(1) = Rmb * Eigen::Vector3d::UnitX();
      J.col(2) = Rmb * Eigen::Vector3d::UnitY();
      const int ix = int(std::floor(q.x() * map.inv())), iy = int(std::floor(q.y() * map.inv())), iz = int(std::floor(q.z() * map.inv()));
      double best = 0, best_dist = 0;
      for (const auto& nb : kNb) {
        const auto* v = map.find(ix + nb[0], iy + nb[1], iz + nb[2]);
        if (!v) continue;
        const Eigen::Vector3d res = q - v->mean, Sr = v->icov_normal * res;
        const double d2 = res.dot(Sr);
        if (d2 > 60.0) continue;
        const double w = std::exp(-0.5 * score_scale * d2);
        H += w * J.transpose() * v->icov_normal * J;
        g += w * J.transpose() * Sr;
        if (w > best) best = w, best_dist = std::abs(v->normal.dot(res));
      }
      if (best > 0.05) sse += best_dist * best_dist, ++r.matched;
    }
  }
  if (r.matched < 50) return r;
  // The normal-only information leaves some directions weak on a plane: a little damping keeps the step sane.
  Eigen::Matrix3d A = H;
  A.diagonal() += 1e-3 * H.diagonal() + Eigen::Vector3d::Constant(1e-6);
  const Eigen::Vector3d dx = -A.ldlt().solve(g);
  r.step = dx.norm();
  r.T.linear() = Eigen::AngleAxisd(dx[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() * T.linear();
  r.T.translation() = T.translation() + Eigen::Vector3d(dx[1], dx[2], 0.0);
  r.rms = std::sqrt(sse / r.matched);  // point-to-plane residual [m]
  return r;
}

// One damped Gauss-Newton step on (rx, ry, rz, x, y, z), all keyframes jointly.
// Rotate in the base frame about the sensor position: R <- Exp(dr) R, t <- t + dt.
inline StepResult refineStep6(const SurfelMap& map, const std::vector<Keyframe>& kfs, const Eigen::Isometry3d& T,
                              double score_scale = 0.25) {
  StepResult r;
  r.T = T;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  Matrix6d H = Matrix6d::Zero();
  Vector6d g = Vector6d::Zero();
  double sse = 0;
  static const int kNb[7][3] = {{0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const auto& kf : kfs) {
    const Eigen::Matrix3d Rmb = kf.T_map_base.linear();
    for (const auto& p : kf.pts) {
      const Eigen::Vector3d rel = T.linear() * p;
      const Eigen::Vector3d q = kf.T_map_base * (rel + T.translation());
      Eigen::Matrix<double, 3, 6> J;
      J.col(0) = Rmb * Eigen::Vector3d(0.0, -rel.z(), rel.y());
      J.col(1) = Rmb * Eigen::Vector3d(rel.z(), 0.0, -rel.x());
      J.col(2) = Rmb * Eigen::Vector3d(-rel.y(), rel.x(), 0.0);
      J.rightCols<3>() = Rmb;
      const int ix = int(std::floor(q.x() * map.inv())), iy = int(std::floor(q.y() * map.inv())), iz = int(std::floor(q.z() * map.inv()));
      double best = 0, best_dist = 0;
      for (const auto& nb : kNb) {
        const auto* v = map.find(ix + nb[0], iy + nb[1], iz + nb[2]);
        if (!v) continue;
        const Eigen::Vector3d res = q - v->mean, Sr = v->icov_normal * res;
        const double d2 = res.dot(Sr);
        if (d2 > 60.0) continue;
        const double w = std::exp(-0.5 * score_scale * d2);
        H += w * J.transpose() * v->icov_normal * J;
        g += w * J.transpose() * Sr;
        if (w > best) best = w, best_dist = std::abs(v->normal.dot(res));
      }
      if (best > 0.05) sse += best_dist * best_dist, ++r.matched;
    }
  }
  if (r.matched < 50) return r;
  r.rms = std::sqrt(sse / r.matched);
  Matrix6d A = H;
  A.diagonal() += 1e-3 * H.diagonal() + Vector6d::Constant(1e-6);
  const Vector6d dx = -A.ldlt().solve(g);
  if (!dx.allFinite()) return r;
  r.step = dx.norm();
  const Eigen::Vector3d dr = dx.head<3>();
  const double angle = dr.norm();
  if (angle > 0.0) r.T.linear() = Eigen::AngleAxisd(angle, dr / angle).toRotationMatrix() * T.linear();
  r.T.translation() = T.translation() + dx.tail<3>();
  return r;
}

}  // namespace iad::calibration
