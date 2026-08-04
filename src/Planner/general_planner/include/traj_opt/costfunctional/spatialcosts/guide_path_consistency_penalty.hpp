#pragma once

#include <algorithm>
#include <cmath>

namespace cost_functional
{
    template <typename GuidePathT, typename GuideTimesT, typename Vec3T>
    inline bool interpolateGuidePathByTime(const GuidePathT *guide_path,
                                           const GuideTimesT *guide_times,
                                           double query_time,
                                           Vec3T &ref_position,
                                           Vec3T &ref_velocity,
                                           Vec3T &ref_tangent,
                                           bool &inside_time_range)
    {
        if (guide_path == nullptr || guide_times == nullptr ||
            guide_path->size() < 2 || guide_path->size() != guide_times->size() ||
            !std::isfinite(query_time))
        {
            return false;
        }

        const int n = static_cast<int>(guide_path->size());
        const double start_time = (*guide_times)[0];
        const double end_time = (*guide_times)[n - 1];
        if (!std::isfinite(start_time) || !std::isfinite(end_time) ||
            end_time <= start_time)
        {
            return false;
        }

        inside_time_range = query_time >= start_time && query_time <= end_time;
        const double t = std::clamp(query_time, start_time, end_time);
        for (int i = 0; i + 1 < n; ++i)
        {
            const double t0 = (*guide_times)[i];
            const double t1 = (*guide_times)[i + 1];
            if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0)
            {
                continue;
            }
            if (t < t0 || (t > t1 && i + 2 < n))
            {
                continue;
            }

            const double alpha = std::clamp((t - t0) / (t1 - t0), 0.0, 1.0);
            const Vec3T p0 = (*guide_path)[i];
            const Vec3T p1 = (*guide_path)[i + 1];
            if (!p0.allFinite() || !p1.allFinite())
            {
                return false;
            }
            const Vec3T segment = p1 - p0;
            const double segment_norm = segment.norm();
            if (segment_norm <= 1.0e-9)
            {
                continue;
            }
            ref_position = p0 + alpha * (p1 - p0);
            ref_tangent = segment / segment_norm;
            if (inside_time_range)
            {
                ref_velocity = segment / (t1 - t0);
            }
            else
            {
                ref_velocity = Vec3T::Zero();
            }
            return ref_position.allFinite() && ref_velocity.allFinite();
        }

        return false;
    }

    inline double robustTubeCost(double excess,
                                 double weight,
                                 double huber_delta,
                                 double &grad_excess)
    {
        if (excess <= 0.0 || weight <= 0.0)
        {
            grad_excess = 0.0;
            return 0.0;
        }
        if (huber_delta > 1.0e-6)
        {
            const double normalized = excess / huber_delta;
            const double scale = std::sqrt(1.0 + normalized * normalized);
            grad_excess = weight * excess / scale;
            return weight * huber_delta * huber_delta * (scale - 1.0);
        }

        grad_excess = weight * excess;
        return 0.5 * weight * excess * excess;
    }

    template <typename GuidePathT, typename GuideTimesT, typename Vec3T>
    inline double accumulateGuidePathConsistencyPenalty(const GuidePathT *guide_path,
                                                        const GuideTimesT *guide_times,
                                                        double query_time,
                                                        const Vec3T &position,
                                                        double weight,
                                                        double lateral_tube_radius,
                                                        double vertical_tube_radius,
                                                        double huber_delta,
                                                        bool enable_time_gradient,
                                                        Vec3T &grad_position,
                                                        double &grad_time,
                                                        double weight_z_lower = 0.0,
                                                        double z_lower_tolerance = 0.0,
                                                        double *max_violation = nullptr,
                                                        double *cost_log = nullptr,
                                                        double *max_abs_time_grad = nullptr,
                                                        int *out_of_time_range_samples = nullptr,
                                                        double *max_z_lower_violation = nullptr)
    {
        if ((weight <= 0.0 && weight_z_lower <= 0.0) || !position.allFinite())
        {
            return 0.0;
        }

        Vec3T ref_position = Vec3T::Zero();
        Vec3T ref_velocity = Vec3T::Zero();
        Vec3T ref_tangent = Vec3T::Zero();
        bool inside_time_range = true;
        if (!interpolateGuidePathByTime(guide_path,
                                        guide_times,
                                        query_time,
                                        ref_position,
                                        ref_velocity,
                                        ref_tangent,
                                        inside_time_range))
        {
            return 0.0;
        }

        const Vec3T diff = position - ref_position;
        Vec3T guide_grad = Vec3T::Zero();
        lateral_tube_radius = std::max(0.0, lateral_tube_radius);
        // Retained in the API for existing lateral-guide callers. Vertical
        // tracking is now handled exclusively by the one-sided z term below.
        (void)vertical_tube_radius;
        huber_delta = std::max(0.0, huber_delta);

        const double tangent_xy_norm =
            std::hypot(ref_tangent.x(), ref_tangent.y());
        double lateral_x = diff.x();
        double lateral_y = diff.y();
        if (tangent_xy_norm > 1.0e-6)
        {
            const double tx = ref_tangent.x() / tangent_xy_norm;
            const double ty = ref_tangent.y() / tangent_xy_norm;
            const double along = diff.x() * tx + diff.y() * ty;
            lateral_x -= along * tx;
            lateral_y -= along * ty;
        }

        const double lateral_norm = std::hypot(lateral_x, lateral_y);
        const double lateral_excess = lateral_norm - lateral_tube_radius;
        double lateral_grad_excess = 0.0;
        double cost = robustTubeCost(lateral_excess,
                                     weight,
                                     huber_delta,
                                     lateral_grad_excess);
        if (lateral_grad_excess > 0.0 && lateral_norm > 1.0e-9)
        {
            guide_grad.x() += lateral_grad_excess * lateral_x / lateral_norm;
            guide_grad.y() += lateral_grad_excess * lateral_y / lateral_norm;
        }

        // A guide is allowed to climb above its reference to clear an obstacle,
        // but it must not sink below the reference by more than the configured
        // tolerance. This is intentionally one-sided.
        const double z_lower_shortfall =
            ref_position.z() - position.z() - std::max(0.0, z_lower_tolerance);
        double z_lower_grad_excess = 0.0;
        cost += robustTubeCost(z_lower_shortfall,
                               weight_z_lower,
                               huber_delta,
                               z_lower_grad_excess);
        if (z_lower_grad_excess > 0.0)
        {
            // d(ref_z - z - tolerance) / dz = -1. A descent step therefore
            // raises z, exactly matching the lower-bound semantics.
            guide_grad.z() -= z_lower_grad_excess;
        }

        grad_position += guide_grad;
        if (enable_time_gradient && inside_time_range)
        {
            const double before = grad_time;
            grad_time += -guide_grad.dot(ref_velocity);
            if (max_abs_time_grad != nullptr)
            {
                *max_abs_time_grad = std::max(*max_abs_time_grad, std::abs(grad_time - before));
            }
        }
        else if (!inside_time_range && out_of_time_range_samples != nullptr)
        {
            ++(*out_of_time_range_samples);
        }

        if (max_violation != nullptr)
        {
            *max_violation = std::max(*max_violation,
                                      std::max(0.0, lateral_excess));
        }
        if (max_z_lower_violation != nullptr)
        {
            *max_z_lower_violation = std::max(*max_z_lower_violation,
                                               std::max(0.0, z_lower_shortfall));
        }
        if (cost_log != nullptr)
        {
            *cost_log += cost;
        }
        return cost;
    }
} // namespace cost_functional
