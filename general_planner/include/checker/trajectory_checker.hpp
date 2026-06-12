#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <checker/check_result.hpp>
#include <checker/common_checker.hpp>
#include <data_structure/backup_traj.h>
#include <data_structure/exp_traj.h>
#include <general_core/config.hpp>

namespace general_planner::checker {
    struct TrajectoryCheckOptions {
        std::string name{"trajectory"};
        bool require_non_empty{true};
        bool check_position_dynamics{true};
        bool check_yaw_dynamics{false};
        double max_vel{-1.0};
        double max_acc{-1.0};
        double max_jerk{-1.0};
        double max_yaw_rate{-1.0};
        double sample_dt{0.05};
        double limit_margin{1.35};
        double min_piece_duration{1.0e-6};
        double duration_tolerance{0.05};
    };

    inline CheckResult checkTrajectory(const geometry_utils::Trajectory &traj,
                                       TrajectoryCheckOptions options) {
        const std::string &name = options.name;
        if (traj.empty()) {
            return options.require_non_empty
                   ? CheckResult::Reject(name + "_EMPTY", name + " is empty")
                   : CheckResult::Ok();
        }
        if (!std::isfinite(traj.start_WT)) {
            return CheckResult::Reject(name + "_START_WT_NON_FINITE", name + " start_WT is not finite");
        }
        const double total_duration = traj.getTotalDuration();
        if (!std::isfinite(total_duration) || total_duration <= options.min_piece_duration) {
            return CheckResult::Reject(name + "_BAD_DURATION", name + " duration is invalid");
        }
        for (int i = 0; i < traj.getPieceNum(); ++i) {
            const auto &piece = traj[i];
            if (!std::isfinite(piece.getDuration()) || piece.getDuration() <= options.min_piece_duration) {
                return CheckResult::Reject(name + "_BAD_PIECE_DURATION",
                                           name + " has invalid piece duration at " + std::to_string(i));
            }
            if (!piece.getCoeffMat().allFinite()) {
                return CheckResult::Reject(name + "_COEFF_NON_FINITE",
                                           name + " has NaN/Inf coefficient at piece " + std::to_string(i));
            }
        }

        double dt = std::max(1.0e-3, options.sample_dt);
        const int max_samples = 2000;
        if (total_duration / dt > max_samples) {
            dt = total_duration / static_cast<double>(max_samples);
        }
        const auto overLimit = [](double value, double limit, double margin) {
            return limit > 0.0 && std::isfinite(limit) && value > limit * margin + 1.0e-6;
        };

        for (double t = 0.0; t <= total_duration + 1.0e-9; t += dt) {
            const double eval_t = std::min(t, total_duration);
            const auto pos = traj.getPos(eval_t);
            const auto vel = traj.getVel(eval_t);
            const auto acc = traj.getAcc(eval_t);
            const auto jer = traj.getJer(eval_t);
            if (!pos.allFinite() || !vel.allFinite() || !acc.allFinite() || !jer.allFinite()) {
                return CheckResult::Reject(name + "_STATE_NON_FINITE",
                                           name + " sampled state is NaN/Inf at t=" + std::to_string(eval_t));
            }
            if (options.check_position_dynamics) {
                if (overLimit(vel.norm(), options.max_vel, options.limit_margin)) {
                    return CheckResult::Reject(name + "_VEL_LIMIT",
                                               name + " velocity exceeds limit at t=" + std::to_string(eval_t));
                }
                if (overLimit(acc.norm(), options.max_acc, options.limit_margin)) {
                    return CheckResult::Reject(name + "_ACC_LIMIT",
                                               name + " acceleration exceeds limit at t=" + std::to_string(eval_t));
                }
                if (overLimit(jer.norm(), options.max_jerk, options.limit_margin)) {
                    return CheckResult::Reject(name + "_JERK_LIMIT",
                                               name + " jerk exceeds limit at t=" + std::to_string(eval_t));
                }
            }
            if (options.check_yaw_dynamics &&
                overLimit(vel.norm(), options.max_yaw_rate, options.limit_margin)) {
                return CheckResult::Reject(name + "_YAW_RATE_LIMIT",
                                           name + " yaw rate exceeds limit at t=" + std::to_string(eval_t));
            }
        }
        return CheckResult::Ok();
    }

    inline CheckResult checkTrajectoryPair(const geometry_utils::Trajectory &pos,
                                           const geometry_utils::Trajectory &yaw,
                                           const Config &cfg,
                                           const std::string &name) {
        TrajectoryCheckOptions pos_options;
        pos_options.name = name + "_pos";
        pos_options.max_vel = cfg.exp_traj_cfg.max_vel;
        pos_options.max_acc = cfg.exp_traj_cfg.max_acc;
        pos_options.max_jerk = cfg.exp_traj_cfg.max_jerk;
        pos_options.sample_dt = std::max(0.02, cfg.sample_traj_dt);
        if (cfg.plain_traj_en && cfg.plain_traj_cfg.max_acc > 0.0) {
            constexpr double kPlainCommitAccRejectRatio = 2.1;
            pos_options.max_acc = cfg.plain_traj_cfg.max_acc *
                                  kPlainCommitAccRejectRatio /
                                  pos_options.limit_margin;
        }
        auto pos_result = checkTrajectory(pos, pos_options);
        if (pos_result.rejected()) {
            return pos_result;
        }

        TrajectoryCheckOptions yaw_options;
        yaw_options.name = name + "_yaw";
        yaw_options.check_position_dynamics = false;
        yaw_options.check_yaw_dynamics = true;
        yaw_options.max_yaw_rate = cfg.yaw_dot_max;
        yaw_options.sample_dt = std::max(0.02, cfg.sample_traj_dt);
        auto yaw_result = checkTrajectory(yaw, yaw_options);
        if (yaw_result.rejected()) {
            return yaw_result;
        }

        const double duration_error = std::abs(pos.getTotalDuration() - yaw.getTotalDuration());
        const double tolerance = std::max(0.05, cfg.sample_traj_dt * 2.0);
        if (duration_error > tolerance) {
            return CheckResult::Reject(name + "_POS_YAW_DURATION_MISMATCH",
                                       name + " pos/yaw duration mismatch: " + std::to_string(duration_error));
        }
        return CheckResult::Ok();
    }

    inline CheckResult checkExpTrajectory(const ExpTraj &exp,
                                          const Config &cfg,
                                          const std::string &name) {
        if (exp.empty()) {
            return CheckResult::Reject(name + "_EMPTY", name + " ExpTraj is empty");
        }
        return checkTrajectoryPair(exp.posTraj(), exp.yawTraj(), cfg, name);
    }

    inline CheckResult checkBackupTrajectory(const BackupTraj &backup,
                                             const Config &cfg,
                                             const std::string &name) {
        if (backup.empty()) {
            return CheckResult::Reject(name + "_EMPTY", name + " BackupTraj is empty");
        }
        if (!std::isfinite(backup.getStartTT()) || backup.getStartTT() < -1.0e-6) {
            return CheckResult::Reject(name + "_BAD_START_TT", name + " backup start TT is invalid");
        }
        return checkTrajectoryPair(backup.posTraj(), backup.yawTraj(), cfg, name);
    }

    inline CheckResult checkExpBackupCommit(const ExpTraj &exp,
                                            const BackupTraj &backup,
                                            const Config &cfg,
                                            const std::string &name) {
        auto exp_result = checkExpTrajectory(exp, cfg, name + "_exp");
        if (exp_result.rejected()) {
            return exp_result;
        }
        auto backup_result = checkBackupTrajectory(backup, cfg, name + "_backup");
        if (backup_result.rejected()) {
            return backup_result;
        }
        const double exp_duration = exp.getTotalDuration();
        const double backup_start_tt = backup.getStartTT();
        if (backup_start_tt <= 1.0e-6 || backup_start_tt >= exp_duration - 1.0e-6) {
            return CheckResult::Reject(name + "_BACKUP_START_OUT_OF_RANGE",
                                       name + " backup start TT is outside exp trajectory duration");
        }
        geometry_utils::Trajectory partial_pos;
        geometry_utils::Trajectory partial_yaw;
        if (!exp.getPartialTrajectoryByTrajectoryTime(0.0, backup_start_tt, partial_pos, partial_yaw)) {
            return CheckResult::Reject(name + "_EXP_PARTIAL_FAIL",
                                       name + " failed to extract exp prefix before backup");
        }
        return CheckResult::Ok();
    }
}
