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

#pragma once

#include <data_structure/base/ellipsoid.h>

namespace optimization_utils {

    using geometry_utils::Ellipsoid;

    struct HomMVIEOptions {
        double softmax_alpha{50.0};
        double unit_sphere_penalty{500.0};
        double min_diagonal{1.0e-7};
        double feasibility_tolerance{1.0e-5};
        int max_iterations{0};
        bool normalize_variables{true};
        bool unit_sphere_constraint{true};
    };

    class HomMVIE {
    public:
        HomMVIE() = default;
        ~HomMVIE() = default;

        static bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                       Ellipsoid &ellipsoid,
                                       const HomMVIEOptions &options = HomMVIEOptions(),
                                       int *lbfgs_iterations = nullptr);

    private:
        struct CostData {
            Eigen::MatrixX3d A;
            double softmax_alpha{50.0};
            double unit_sphere_penalty{500.0};
            double min_diagonal{1.0e-7};
            bool unit_sphere_constraint{true};
            int lbfgs_iterations{0};
        };

        static double costMVIEHomOpt(void *data,
                                     const Eigen::VectorXd &y,
                                     Eigen::VectorXd &grad);

        static int recordProgress(void *data,
                                  const Eigen::VectorXd &y,
                                  const Eigen::VectorXd &grad,
                                  double fx,
                                  double step,
                                  int k,
                                  int ls);

        static bool ellipsoidSatisfiesHalfspaces(const Eigen::MatrixX4d &hPoly,
                                                 const Eigen::Matrix3d &R,
                                                 const Eigen::Vector3d &r,
                                                 const Eigen::Vector3d &p,
                                                 double tolerance);
    };
}
