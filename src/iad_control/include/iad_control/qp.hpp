// Small dense QP solver: min 1/2 x'Px + q'x  s.t.  l <= Ax <= u, by ADMM (the OSQP iteration, with one
// LDLT factorisation per solve). Enough for MPC horizons of a few dozen inputs; warm-startable. ROS-free.
#pragma once
#include <Eigen/Dense>

namespace iad::control {

struct QpResult {
  Eigen::VectorXd x;
  int iterations = 0;
  bool converged = false;
  double primal_res = 0, dual_res = 0;
};

inline QpResult solveQp(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& A, const Eigen::VectorXd& l,
                        const Eigen::VectorXd& u, const Eigen::VectorXd* warm = nullptr, int max_iter = 400,
                        double eps = 1e-5, double rho = 0.1, double sigma = 1e-6, double alpha = 1.6) {
  const int n = int(P.rows());
  QpResult r;
  r.x = warm && warm->size() == n ? *warm : Eigen::VectorXd::Zero(n);
  Eigen::VectorXd z = (A * r.x).cwiseMax(l).cwiseMin(u), y = Eigen::VectorXd::Zero(A.rows());
  const Eigen::MatrixXd K = P + sigma * Eigen::MatrixXd::Identity(n, n) + rho * A.transpose() * A;
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(K);
  for (int it = 0; it < max_iter; ++it) {
    const Eigen::VectorXd xt = ldlt.solve(sigma * r.x - q + A.transpose() * (rho * z - y));
    const Eigen::VectorXd zt = A * xt;
    const Eigen::VectorXd x_new = alpha * xt + (1 - alpha) * r.x;
    const Eigen::VectorXd zr = alpha * zt + (1 - alpha) * z;
    const Eigen::VectorXd z_new = (zr + y / rho).cwiseMax(l).cwiseMin(u);
    y += rho * (zr - z_new);
    r.dual_res = (rho * A.transpose() * (z_new - z)).lpNorm<Eigen::Infinity>();
    r.x = x_new, z = z_new;
    r.primal_res = (A * r.x - z).lpNorm<Eigen::Infinity>();
    r.iterations = it + 1;
    if (r.primal_res < eps && r.dual_res < eps) {
      r.converged = true;
      break;
    }
  }
  return r;
}

}  // namespace iad::control
