// NDT scan matching against a voxelised point-cloud map (the role of Autoware's ndt_scan_matcher). ROS-free.
// Planar voxels are used point-to-plane (as in VGICP-style matchers), others as full Gaussians.
//
// Map: one Gaussian (mean, regularised inverse covariance) per voxel. Alignment: Gauss-Newton on the
// Mahalanobis distance of every source point to the Gaussians of its voxel and 6 face neighbours
// (DIRECT7), each term weighted by exp(-d^2/2) as in Magnusson's NDT score so far-off Gaussians and
// unmapped objects fade out. Rotation is linearised about the vehicle, not the map origin (map
// coordinates are hundreds of metres; rotating about the origin conditions the problem badly).
// The Gauss-Newton Hessian gives the pose covariance: it is large along directions the geometry does
// not constrain (an open quay is a plane: x/y/yaw float), which is exactly what the EKF needs to know.
#pragma once
#include <omp.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "iad_localization/util.hpp"

namespace iad::localization {

struct VoxelGaussian {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d icov = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d icov_normal = Eigen::Matrix3d::Zero();  // information along the surface normal only
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  bool planar = false;
};

class NdtMap {
 public:
  // min_std: floor on every axis of a voxel Gaussian. Thin (planar) Gaussians give a precise fit but a
  // narrow basin; the coarse pyramid level uses a fat floor to pull from ~1 m away.
  void build(const std::vector<Eigen::Vector3d>& pts, double res, double min_std = 0.1, int min_pts = 6) {
    res_ = res, inv_ = 1.0 / res;
    struct Acc { Eigen::Vector3d s = Eigen::Vector3d::Zero(); Eigen::Matrix3d ss = Eigen::Matrix3d::Zero(); int n = 0; };
    std::unordered_map<int64_t, Acc> acc;
    for (const auto& p : pts) {
      auto& a = acc[voxelOf(p, inv_)];
      a.s += p, a.ss += p * p.transpose(), ++a.n;
    }
    vox_.clear();
    vox_.reserve(acc.size());
    for (const auto& kv : acc) {
      const Acc& a = kv.second;
      if (a.n < min_pts) continue;
      VoxelGaussian g;
      g.mean = a.s / a.n;
      Eigen::Matrix3d C = (a.ss - a.n * g.mean * g.mean.transpose()) / (a.n - 1);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
      Eigen::Vector3d ev = es.eigenvalues();
      const double lmax = ev.maxCoeff();
      for (int k = 0; k < 3; ++k) ev[k] = std::max({ev[k], 0.01 * lmax, min_std * min_std});
      g.icov = es.eigenvectors() * ev.cwiseInverse().asDiagonal() * es.eigenvectors().transpose();
      // For the covariance estimate keep only what the surface constrains: a planar voxel says nothing
      // about sliding along itself (otherwise a flat ground "locks" every direction to the voxel grid).
      const Eigen::Vector3d nrm = es.eigenvectors().col(0);
      g.normal = nrm, g.planar = ev[0] * 10 < ev[1];
      g.icov_normal = g.planar ? Eigen::Matrix3d(nrm * nrm.transpose() / ev[0]) : g.icov;
      vox_.emplace(kv.first, g);
    }
  }
  const VoxelGaussian* find(int ix, int iy, int iz) const {
    auto it = vox_.find(packVoxel(ix, iy, iz));
    return it == vox_.end() ? nullptr : &it->second;
  }
  double inv() const { return inv_; }
  size_t size() const { return vox_.size(); }

 private:
  std::unordered_map<int64_t, VoxelGaussian> vox_;
  double res_ = 2.0, inv_ = 0.5;
};

struct NdtParams {
  int max_iter = 30;
  bool point_to_plane = true;  // planar voxels contribute only along their normal (no in-plane grid-locking)
  double score_scale = 0.25;  // w = exp(-score_scale * d^2 / 2): broader than the voxel Gaussian (ndt_omp's d2)
  double max_d2 = 120.0;
  double eps_trans = 0.002, eps_rot = 2e-5;
  double lambda = 1e-6;  // Levenberg damping (relative to diag(H))
  double cov_scale = 1.0, min_std_xy = 0.03, min_std_yaw = 0.002;
};

struct NdtResult {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();      // order: rot(x,y,z), trans(x,y,z)
  Eigen::Matrix<double, 6, 6> H_cov = Eigen::Matrix<double, 6, 6>::Zero();  // degeneracy-aware information
  Eigen::Matrix3d cov_xyyaw = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d cov_raw = Eigen::Matrix3d::Identity();  // before the minimum-std floor
  double nvtl = 0;  // mean per-point best exp(-d^2/2): 1 = perfect fit
  int iterations = 0, matched = 0;
  bool converged = false;
};

// src: points in the vehicle (base_link) frame.
inline NdtResult alignNdt(const NdtMap& map, const std::vector<Eigen::Vector3d>& src, const Eigen::Isometry3d& init,
                          const NdtParams& prm) {
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  static const int kNb[7][3] = {{0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  NdtResult res;
  res.T = init;
  const int nthreads = std::max(1, std::min(8, omp_get_max_threads()));
  for (int it = 0; it < prm.max_iter; ++it) {
    const Eigen::Matrix3d R = res.T.linear();
    const Eigen::Vector3d c = res.T.translation();
    std::vector<Mat6> Hs(size_t(nthreads), Mat6::Zero()), Hc(size_t(nthreads), Mat6::Zero());
    std::vector<Vec6> gs(size_t(nthreads), Vec6::Zero());
    std::vector<double> score(size_t(nthreads), 0.0);
    std::vector<int> matched(size_t(nthreads), 0);
#pragma omp parallel for num_threads(nthreads) schedule(static)
    for (int i = 0; i < int(src.size()); ++i) {
      const int t = omp_get_thread_num();
      const Eigen::Vector3d q = R * src[size_t(i)] + c, ql = q - c;
      const int ix = int(std::floor(q.x() * map.inv())), iy = int(std::floor(q.y() * map.inv())), iz = int(std::floor(q.z() * map.inv()));
      Eigen::Matrix<double, 3, 6> J;
      J.leftCols<3>() = -skew(ql);
      J.rightCols<3>().setIdentity();
      double best = 0;
      for (const auto& nb : kNb) {
        const VoxelGaussian* g = map.find(ix + nb[0], iy + nb[1], iz + nb[2]);
        if (!g) continue;
        const Eigen::Matrix3d& S = prm.point_to_plane ? g->icov_normal : g->icov;
        const Eigen::Vector3d r = q - g->mean;
        const Eigen::Vector3d Sr = S * r;
        const double d2 = r.dot(Sr);
        if (d2 > prm.max_d2) continue;
        const double w = std::exp(-0.5 * prm.score_scale * d2);
        best = std::max(best, w);
        const Eigen::Matrix<double, 3, 6> SJ = S * J;
        Hs[size_t(t)].noalias() += w * J.transpose() * SJ;
        Hc[size_t(t)].noalias() += w * J.transpose() * (g->icov_normal * J);
        gs[size_t(t)].noalias() += w * J.transpose() * Sr;
      }
      score[size_t(t)] += best;
      matched[size_t(t)] += best > 0.05;
    }
    Mat6 H = Mat6::Zero(), Hcv = Mat6::Zero();
    Vec6 g = Vec6::Zero();
    for (int t = 0; t < nthreads; ++t) Hcv += Hc[size_t(t)];
    res.H_cov = Hcv;
    double sc = 0;
    int m = 0;
    for (int t = 0; t < nthreads; ++t) H += Hs[size_t(t)], g += gs[size_t(t)], sc += score[size_t(t)], m += matched[size_t(t)];
    res.H = H, res.matched = m, res.nvtl = src.empty() ? 0.0 : sc / double(src.size());
    res.iterations = it + 1;
    if (m < 20) break;
    Mat6 A = H;
    A.diagonal() += prm.lambda * H.diagonal() + Vec6::Constant(1e-9);
    const Vec6 dx = -A.ldlt().solve(g);
    // Apply: rotate about the vehicle position c, then translate.
    Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
    D.linear() = expSE3((Vec6() << dx.head<3>(), Eigen::Vector3d::Zero()).finished()).linear();
    Eigen::Isometry3d Tn = Eigen::Isometry3d::Identity();
    Tn.linear() = D.linear() * R;
    Tn.translation() = c + dx.tail<3>();
    res.T = Tn;
    if (dx.tail<3>().norm() < prm.eps_trans && dx.head<3>().norm() < prm.eps_rot) {
      res.converged = true;
      break;
    }
  }
  // Covariance of (x, y, yaw) from the information matrix (marginalise the 6-DoF inverse).
  Eigen::Matrix<double, 6, 6> Hreg = res.H_cov;
  Hreg.diagonal() += Eigen::Matrix<double, 6, 1>::Constant(1e-3);
  const Eigen::Matrix<double, 6, 6> C = prm.cov_scale * Hreg.inverse();
  res.cov_xyyaw << C(3, 3), C(3, 4), C(3, 2), C(4, 3), C(4, 4), C(4, 2), C(2, 3), C(2, 4), C(2, 2);
  res.cov_raw = res.cov_xyyaw;
  res.cov_xyyaw(0, 0) = std::max(res.cov_xyyaw(0, 0), sq(prm.min_std_xy));
  res.cov_xyyaw(1, 1) = std::max(res.cov_xyyaw(1, 1), sq(prm.min_std_xy));
  res.cov_xyyaw(2, 2) = std::max(res.cov_xyyaw(2, 2), sq(prm.min_std_yaw));
  return res;
}

// Coarse-to-fine: a few iterations on the coarse map widen the basin, the fine map gives the pose and
// the covariance.
struct NdtPyramid {
  NdtMap coarse, fine;
  void build(const std::vector<Eigen::Vector3d>& pts, double fine_res = 2.0) {
    coarse.build(pts, 2.0 * fine_res, 0.25);
    fine.build(pts, fine_res, 0.10, 4);  // 4 points: a 1 m-sampled survey still fills every 2 m ground voxel
  }
  NdtResult align(const std::vector<Eigen::Vector3d>& src, const Eigen::Isometry3d& init, NdtParams prm = {}) const {
    NdtParams pc = prm;
    pc.max_iter = 12, pc.eps_trans = 0.02, pc.eps_rot = 2e-4;
    const NdtResult rc = alignNdt(coarse, src, init, pc);
    NdtResult rf = alignNdt(fine, src, rc.matched > 20 ? rc.T : init, prm);
    rf.iterations += rc.iterations;
    return rf;
  }
};

}  // namespace iad::localization
