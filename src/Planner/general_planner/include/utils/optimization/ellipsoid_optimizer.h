#pragma once

#include <data_structure/base/ellipsoid.h>
#include <utils/optimization/hom_mvie.h>

#include <string>

namespace optimization_utils {

    using geometry_utils::Ellipsoid;

    enum class EllipsoidOptimizerBackend {
        CLASSIC,
        HOM,
        HOM_NO_NORMALIZATION,
        HOM_WITH_CLASSIC_FALLBACK
    };

    struct EllipsoidOptimizerConfig {
        EllipsoidOptimizerBackend backend{EllipsoidOptimizerBackend::CLASSIC};
        HomMVIEOptions hom_options{};
    };

    class EllipsoidOptimizer {
    public:
        EllipsoidOptimizer() = default;
        ~EllipsoidOptimizer() = default;

        static EllipsoidOptimizerConfig makeConfig(const std::string &backend_name,
                                                   bool fallback_to_classic);

        static bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                       Ellipsoid &ellipsoid,
                                       const EllipsoidOptimizerConfig &config,
                                       int *lbfgs_iterations = nullptr);

        static const char *backendName(EllipsoidOptimizerBackend backend);
    };
}
