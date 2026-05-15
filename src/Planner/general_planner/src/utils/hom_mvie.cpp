/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)
    Modifications for Hom-Opt MVIE by KaiChen Guo(kaireriemann2025@163.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include <utils/optimization/hom_mvie.h>
#include <utils/optimization/lbfgs.h>
#include <utils/optimization/mvie.h>
#include <utils/optimization/sdlp.h>

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace optimization_utils {

    using geometry_utils::Ellipsoid;
    using math_utils::lbfgs;
    namespace sdlp = math_utils::sdlp;

    double HomMVIE::costMVIEHomOpt(void *data,
                                   const Eigen::VectorXd &y,
                                   Eigen::VectorXd &grad) {
        const auto *optData = static_cast<const CostData *>(data);
        const int M = optData->A.rows();
        const Eigen::MatrixX3d &A = optData->A;
        const double alpha = std::max(optData->softmax_alpha, 1.0e-6);
        const double lambda = std::max(optData->unit_sphere_penalty, 1.0e-6);
        const double min_diagonal = std::max(optData->min_diagonal, DBL_EPSILON);

        grad.setZero();

        if (y(3) <= min_diagonal || y(4) <= min_diagonal || y(5) <= min_diagonal) {
            double cost = 1.0e6;
            if (y(3) <= min_diagonal) {
                cost += 1.0e5 * (min_diagonal - y(3));
                grad(3) = -1.0e5;
            }
            if (y(4) <= min_diagonal) {
                cost += 1.0e5 * (min_diagonal - y(4));
                grad(4) = -1.0e5;
            }
            if (y(5) <= min_diagonal) {
                cost += 1.0e5 * (min_diagonal - y(5));
                grad(5) = -1.0e5;
            }
            return cost;
        }

        Eigen::VectorXd mu(M);
        Eigen::VectorXd n(M);
        Eigen::MatrixX3d v(M, 3);
        for (int i = 0; i < M; ++i) {
            v(i, 0) = A(i, 0) * y(3) + A(i, 1) * y(6) + A(i, 2) * y(8);
            v(i, 1) = A(i, 1) * y(4) + A(i, 2) * y(7);
            v(i, 2) = A(i, 2) * y(5);
            n(i) = std::sqrt(v.row(i).squaredNorm() + 1.0e-12);
            mu(i) = n(i) + A(i, 0) * y(0) + A(i, 1) * y(1) + A(i, 2) * y(2);
        }

        const double mu_max = mu.maxCoeff();
        const Eigen::VectorXd exp_term = (alpha * (mu.array() - mu_max)).exp();
        const double sum_exp = exp_term.sum();
        const double mu_val = mu_max + std::log(sum_exp) / alpha;
        if (!(mu_val > 0.0) || !std::isfinite(mu_val)) {
            grad = 1.0e3 * y;
            return 1.0e9;
        }
        const Eigen::VectorXd w = exp_term / sum_exp;

        double cost = -std::log(y(3)) - std::log(y(4)) - std::log(y(5)) + 3.0 * std::log(mu_val);

        Eigen::VectorXd grad_mu = Eigen::VectorXd::Zero(9);
        for (int i = 0; i < M; ++i) {
            const double inv_n = 1.0 / n(i);

            grad_mu(0) += w(i) * A(i, 0);
            grad_mu(1) += w(i) * A(i, 1);
            grad_mu(2) += w(i) * A(i, 2);
            grad_mu(3) += w(i) * v(i, 0) * inv_n * A(i, 0);
            grad_mu(4) += w(i) * v(i, 1) * inv_n * A(i, 1);
            grad_mu(5) += w(i) * v(i, 2) * inv_n * A(i, 2);
            grad_mu(6) += w(i) * v(i, 0) * inv_n * A(i, 1);
            grad_mu(7) += w(i) * v(i, 1) * inv_n * A(i, 2);
            grad_mu(8) += w(i) * v(i, 0) * inv_n * A(i, 2);
        }

        grad = (3.0 / mu_val) * grad_mu;
        grad(3) -= 1.0 / y(3);
        grad(4) -= 1.0 / y(4);
        grad(5) -= 1.0 / y(5);

        if (optData->unit_sphere_constraint) {
            const double norm_y_sqr = y.squaredNorm();
            const double diff = norm_y_sqr - 1.0;
            cost += 0.5 * lambda * diff * diff;
            grad += 2.0 * lambda * diff * y;
        }

        return cost;
    }

    int HomMVIE::recordProgress(void *data,
                                const Eigen::VectorXd &,
                                const Eigen::VectorXd &,
                                double,
                                double,
                                int k,
                                int) {
        auto *optData = static_cast<CostData *>(data);
        if (optData != nullptr) {
            optData->lbfgs_iterations = k;
        }
        return 0;
    }

    bool HomMVIE::ellipsoidSatisfiesHalfspaces(const Eigen::MatrixX4d &hPoly,
                                               const Eigen::Matrix3d &R,
                                               const Eigen::Vector3d &r,
                                               const Eigen::Vector3d &p,
                                               double tolerance) {
        if (!R.allFinite() || !r.allFinite() || !p.allFinite() || (r.array() <= 0.0).any()) {
            return false;
        }

        const Eigen::Matrix3d C = R * r.asDiagonal();
        for (int i = 0; i < hPoly.rows(); ++i) {
            const Eigen::Vector3d normal = hPoly.block<1, 3>(i, 0).transpose();
            const double support = normal.dot(p) + hPoly(i, 3) + (C.transpose() * normal).norm();
            if (!std::isfinite(support) || support > tolerance) {
                return false;
            }
        }
        return true;
    }

    bool HomMVIE::maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                     Ellipsoid &ellipsoid,
                                     const HomMVIEOptions &options,
                                     int *lbfgs_iterations) {
        if (lbfgs_iterations != nullptr) {
            *lbfgs_iterations = 0;
        }
        const int M = hPoly.rows();
        if (M <= 0 || hPoly.cols() != 4 || !hPoly.allFinite()) {
            return false;
        }

        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        if ((hNorm <= DBL_EPSILON).any()) {
            return false;
        }

        Eigen::MatrixX4d Alp(M, 4);
        Eigen::VectorXd blp(M);
        Eigen::Vector4d clp, xlp;
        Alp.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        Alp.rightCols<1>().setConstant(1.0);
        blp = -hPoly.rightCols<1>().array() / hNorm;
        clp.setZero();
        clp(3) = -1.0;

        const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
        if (!(maxdepth > 0.0) || std::isinf(maxdepth) || !xlp.allFinite()) {
            return false;
        }
        const Eigen::Vector3d interior = xlp.head<3>();

        const Eigen::VectorXd denom = blp - Alp.leftCols<3>() * interior;
        if ((denom.array() <= DBL_EPSILON).any() || !denom.allFinite()) {
            return false;
        }

        CostData optData;
        optData.A = Alp.leftCols<3>().array().colwise() / denom.array();
        optData.softmax_alpha = options.softmax_alpha;
        optData.unit_sphere_penalty = options.unit_sphere_penalty;
        optData.min_diagonal = options.min_diagonal;
        optData.unit_sphere_constraint = options.unit_sphere_constraint;

        Eigen::Matrix3d R = ellipsoid.R();
        Eigen::Vector3d r = ellipsoid.r();
        Eigen::Vector3d p = ellipsoid.d();
        if (!R.allFinite() || !r.allFinite() || !p.allFinite() || (r.array() <= 0.0).any()) {
            return false;
        }

        const Eigen::Matrix3d Q = R * r.cwiseProduct(r).asDiagonal() * R.transpose();
        Eigen::Matrix3d L;
        MVIE::chol3d(Q, L);
        if (!L.allFinite() || L(0, 0) <= 0.0 || L(1, 1) <= 0.0 || L(2, 2) <= 0.0) {
            return false;
        }

        Eigen::VectorXd y(9);
        y.head<3>() = p - interior;
        y(3) = L(0, 0);
        y(4) = L(1, 1);
        y(5) = L(2, 2);
        y(6) = L(1, 0);
        y(7) = L(2, 1);
        y(8) = L(2, 0);

        if (options.normalize_variables) {
            if (y.norm() > 1.0e-6) {
                y.normalize();
            } else {
                y.setZero();
                y(3) = 1.0;
                y(4) = 1.0;
                y(5) = 1.0;
                y.normalize();
            }
        }

        double minCost = 0.0;
        lbfgs::lbfgs_parameter_t paramsMVIE;
        paramsMVIE.mem_size = 18;
        paramsMVIE.g_epsilon = 0.0;
        paramsMVIE.min_step = 1.0e-32;
        paramsMVIE.past = 3;
        paramsMVIE.delta = 1.0e-2;
        paramsMVIE.max_iterations = options.max_iterations;

        const int ret = lbfgs::lbfgs_optimize(y,
                                              minCost,
                                              &HomMVIE::costMVIEHomOpt,
                                              nullptr,
                                              &HomMVIE::recordProgress,
                                              &optData,
                                              paramsMVIE);
        if (lbfgs_iterations != nullptr) {
            *lbfgs_iterations = optData.lbfgs_iterations;
        }
        if (ret < 0 || !y.allFinite()) {
            return false;
        }

        Eigen::VectorXd mu_vec(M);
        for (int i = 0; i < M; ++i) {
            const double v0 = optData.A(i, 0) * y(3) + optData.A(i, 1) * y(6) + optData.A(i, 2) * y(8);
            const double v1 = optData.A(i, 1) * y(4) + optData.A(i, 2) * y(7);
            const double v2 = optData.A(i, 2) * y(5);
            const double norm_v = std::sqrt(v0 * v0 + v1 * v1 + v2 * v2 + 1.0e-12);
            mu_vec(i) = norm_v + optData.A(i, 0) * y(0) + optData.A(i, 1) * y(1) + optData.A(i, 2) * y(2);
        }

        const double final_mu = mu_vec.maxCoeff();
        if (!(final_mu > DBL_EPSILON) || !std::isfinite(final_mu)) {
            return false;
        }
        const Eigen::VectorXd x = y / final_mu;

        p = x.head<3>() + interior;
        L.setZero();
        L(0, 0) = x(3);
        L(1, 0) = x(6);
        L(1, 1) = x(4);
        L(2, 0) = x(8);
        L(2, 1) = x(7);
        L(2, 2) = x(5);
        if (!L.allFinite() || L(0, 0) <= 0.0 || L(1, 1) <= 0.0 || L(2, 2) <= 0.0) {
            return false;
        }

        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(L, Eigen::ComputeFullU);
        const Eigen::Matrix3d U = svd.matrixU();
        const Eigen::Vector3d S = svd.singularValues();
        if (!U.allFinite() || !S.allFinite() || (S.array() <= 0.0).any()) {
            return false;
        }

        if (U.determinant() < 0.0) {
            R.col(0) = U.col(1);
            R.col(1) = U.col(0);
            R.col(2) = U.col(2);
            r(0) = S(1);
            r(1) = S(0);
            r(2) = S(2);
        } else {
            R = U;
            r = S;
        }

        if (!ellipsoidSatisfiesHalfspaces(hPoly, R, r, p, options.feasibility_tolerance)) {
            return false;
        }

        ellipsoid = Ellipsoid(R, r, p);
        return true;
    }
}
