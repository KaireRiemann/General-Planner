#pragma once

#include "utils/optimization/lbfgs.h"
#include <eigen3/Eigen/Eigen>
#include <cfloat>

namespace cost_functional_manager
{
    class PolytopeProjectionSolverAdapter
    {
    public:
        using CostFunction = double (*)(void *instance,
                                        const Eigen::VectorXd &x,
                                        Eigen::VectorXd &g);

        static inline void optimize(Eigen::VectorXd &x,
                                    double &min_cost,
                                    CostFunction cost_function,
                                    void *instance)
        {
            math_utils::lbfgs::lbfgs_parameter_t params;
            params.past = 0;
            params.delta = 1.0e-5;
            params.g_epsilon = FLT_EPSILON;
            params.max_iterations = 128;
            math_utils::lbfgs::lbfgs_optimize(x,
                                            min_cost,
                                            cost_function,
                                            nullptr,
                                            nullptr,
                                            instance,
                                            params);
        }
    };
}// namespace cost_functional_manager