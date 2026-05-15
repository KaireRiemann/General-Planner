#include <utils/optimization/ellipsoid_optimizer.h>
#include <utils/optimization/mvie.h>

#include <algorithm>
#include <cctype>

namespace optimization_utils {

    namespace {
        std::string normalizedName(std::string name) {
            std::transform(name.begin(), name.end(), name.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            std::replace(name.begin(), name.end(), '-', '_');
            return name;
        }
    }

    EllipsoidOptimizerConfig EllipsoidOptimizer::makeConfig(const std::string &backend_name,
                                                            const bool fallback_to_classic) {
        EllipsoidOptimizerConfig config;
        const std::string name = normalizedName(backend_name);
        if (name == "hom" || name == "hom_mvie" || name == "hom_firi") {
            config.backend = fallback_to_classic ? EllipsoidOptimizerBackend::HOM_WITH_CLASSIC_FALLBACK
                                                 : EllipsoidOptimizerBackend::HOM;
        } else if (name == "hom_no_normalization" || name == "hom_no_norm" ||
                   name == "hom_without_normalization" || name == "hom_firi_no_normalization") {
            config.backend = EllipsoidOptimizerBackend::HOM_NO_NORMALIZATION;
            config.hom_options.normalize_variables = false;
            config.hom_options.unit_sphere_constraint = false;
        } else if (name == "hom_fallback" || name == "hom_mvie_fallback" ||
                   name == "hom_with_classic_fallback") {
            config.backend = EllipsoidOptimizerBackend::HOM_WITH_CLASSIC_FALLBACK;
        } else {
            config.backend = EllipsoidOptimizerBackend::CLASSIC;
        }
        return config;
    }

    bool EllipsoidOptimizer::maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                                Ellipsoid &ellipsoid,
                                                const EllipsoidOptimizerConfig &config,
                                                int *lbfgs_iterations) {
        if (lbfgs_iterations != nullptr) {
            *lbfgs_iterations = 0;
        }
        switch (config.backend) {
            case EllipsoidOptimizerBackend::CLASSIC:
                return MVIE::maxVolInsEllipsoid(hPoly, ellipsoid, lbfgs_iterations);
            case EllipsoidOptimizerBackend::HOM:
                return HomMVIE::maxVolInsEllipsoid(hPoly, ellipsoid, config.hom_options, lbfgs_iterations);
            case EllipsoidOptimizerBackend::HOM_NO_NORMALIZATION:
                return HomMVIE::maxVolInsEllipsoid(hPoly, ellipsoid, config.hom_options, lbfgs_iterations);
            case EllipsoidOptimizerBackend::HOM_WITH_CLASSIC_FALLBACK: {
                int hom_iterations = 0;
                Ellipsoid hom_ellipsoid = ellipsoid;
                if (HomMVIE::maxVolInsEllipsoid(hPoly, hom_ellipsoid, config.hom_options, &hom_iterations)) {
                    if (lbfgs_iterations != nullptr) {
                        *lbfgs_iterations = hom_iterations;
                    }
                    ellipsoid = hom_ellipsoid;
                    return true;
                }
                int classic_iterations = 0;
                const bool ok = MVIE::maxVolInsEllipsoid(hPoly, ellipsoid, &classic_iterations);
                if (lbfgs_iterations != nullptr) {
                    *lbfgs_iterations = hom_iterations + classic_iterations;
                }
                return ok;
            }
        }
        return MVIE::maxVolInsEllipsoid(hPoly, ellipsoid, lbfgs_iterations);
    }

    const char *EllipsoidOptimizer::backendName(const EllipsoidOptimizerBackend backend) {
        switch (backend) {
            case EllipsoidOptimizerBackend::CLASSIC:
                return "classic";
            case EllipsoidOptimizerBackend::HOM:
                return "hom";
            case EllipsoidOptimizerBackend::HOM_NO_NORMALIZATION:
                return "hom_no_normalization";
            case EllipsoidOptimizerBackend::HOM_WITH_CLASSIC_FALLBACK:
                return "hom_fallback";
        }
        return "classic";
    }
}
