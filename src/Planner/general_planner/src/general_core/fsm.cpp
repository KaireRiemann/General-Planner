/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fsm/fsm.h>
#include <checker/common_checker.hpp>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>

using namespace general_utils;

namespace fsm {
    namespace {
        double yawDiff(const double lhs, const double rhs) {
            return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
        }

        std::string retCodeName(const int ret_code) {
            switch (ret_code) {
                case FAILED:
                    return "FAILED";
                case NO_NEED:
                    return "NO_NEED";
                case SUCCESS:
                    return "SUCCESS";
                case FINISH:
                    return "FINISH";
                case NEW_TRAJ:
                    return "NEW_TRAJ";
                case EMER:
                    return "EMER";
                case OPT_FAILED:
                    return "OPT_FAILED";
                case INIT_ERROR:
                    return "INIT_ERROR";
                case REACH_HORIZON:
                    return "REACH_HORIZON";
                case REACH_GOAL:
                    return "REACH_GOAL";
                case NO_PATH:
                    return "NO_PATH";
                case TIME_OUT:
                    return "TIME_OUT";
                default:
                    return std::to_string(ret_code);
            }
        }

        std::string gridTypeName(const int grid_type) {
            switch (grid_type) {
                case rog_map::GridType::UNKNOWN:
                    return "UNKNOWN";
                case rog_map::GridType::OUT_OF_MAP:
                    return "OUT_OF_MAP";
                case rog_map::GridType::OCCUPIED:
                    return "OCCUPIED";
                case rog_map::GridType::KNOWN_FREE:
                    return "KNOWN_FREE";
                default:
                    return std::to_string(grid_type);
            }
        }

        std::string csvEscape(const std::string &value) {
            bool needs_quotes = false;
            std::string escaped;
            escaped.reserve(value.size());
            for (const char c: value) {
                if (c == '"' || c == ',' || c == '\n' || c == '\r') {
                    needs_quotes = true;
                }
                if (c == '"') {
                    escaped.push_back('"');
                }
                escaped.push_back(c);
            }
            if (!needs_quotes) {
                return escaped;
            }
            return "\"" + escaped + "\"";
        }

        void writeDiagnosticCsvHeader(std::ostream &out) {
            out << "stamp,relative_time,replan_id,replan_log_id,level,event,task_mode,machine_state,detail,"
                << "goal_x,goal_y,goal_z,goal_yaw,robot_x,robot_y,robot_z,robot_vx,robot_vy,robot_vz,"
                << "robot_state_received,traj_seq,ret_code,ret_code_name,total_replan_time_ms,"
                << "remaining_traj_s,on_backup"
                << std::endl;
        }

        void writeDiagnosticCsvRow(std::ostream &out,
                                   const Fsm::DiagnosticEvent &event,
                                   const double system_start_time) {
            out << std::fixed << std::setprecision(15)
                << event.stamp << ","
                << event.stamp - system_start_time << ","
                << event.replan_id << ","
                << event.replan_log_id << ","
                << csvEscape(event.level) << ","
                << csvEscape(event.event) << ","
                << csvEscape(event.task_mode) << ","
                << csvEscape(event.machine_state) << ","
                << csvEscape(event.detail) << ","
                << event.goal_p.x() << ","
                << event.goal_p.y() << ","
                << event.goal_p.z() << ","
                << event.goal_yaw << ","
                << event.robot_p.x() << ","
                << event.robot_p.y() << ","
                << event.robot_p.z() << ","
                << event.robot_v.x() << ","
                << event.robot_v.y() << ","
                << event.robot_v.z() << ","
                << static_cast<int>(event.robot_state_received) << ","
                << event.traj_seq << ","
                << event.ret_code << ","
                << csvEscape(retCodeName(event.ret_code)) << ","
                << event.total_replan_time_ms << ","
                << event.remaining_traj_s << ","
                << static_cast<int>(event.on_backup)
                << std::endl;
        }
    }

    Fsm::~Fsm() {
        write_time_.close();
        diagnostic_event_log_.close();
        tracking_diagnostic_event_log_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }

        if (state2stateMode() &&
            cfg_.backend_type == general_planner::architecture::BackendType::CORRIDOR) {
            // Always stamp the configured/last cost mode (ctor + optimize).
            // When this replan skipped ExpTrajOpt, keep mode but zero metrics.
            traj_opt::ExpTrajOpt::TimingReport report =
                    planner_ptr_->getLatestExpTimingReport();
            const std::string configured_mode = report.mode;
            if (!(log_module_time.size() > EXP_TRAJ_OPT &&
                  log_module_time[EXP_TRAJ_OPT] > 0.0)) {
                report = traj_opt::ExpTrajOpt::TimingReport{};
                report.mode = configured_mode.empty() ? "not_run" : configured_mode;
            }
            const double exp_opt_seconds =
                    log_module_time.size() > EXP_TRAJ_OPT
                    ? log_module_time[EXP_TRAJ_OPT]
                    : 0.0;
            const double total_replan_seconds =
                    log_module_time.size() > TOTAL_REPLAN
                    ? log_module_time[TOTAL_REPLAN]
                    : 0.0;
            write_time_ << ", " << report.mode
                        << ", " << report.evaluations
                        << ", " << report.iterations
                        << ", " << report.line_search_evaluations
                        << ", " << (report.iterations > 0
                                      ? static_cast<double>(report.line_search_evaluations) /
                                            static_cast<double>(report.iterations)
                                      : 0.0)
                        << ", " << report.max_line_search_evaluations
                        << ", " << (report.iterations > 0
                                      ? report.accepted_step_sum /
                                            static_cast<double>(report.iterations)
                                      : 0.0)
                        << ", " << report.min_accepted_step
                        << ", " << report.polynomial_pieces
                        << ", " << report.dense_nodes_per_evaluation
                        << ", " << report.hull_control_checks_per_evaluation
                        << ", " << report.scalar_constraint_checks
                        << ", " << report.alm_constraints
                        << ", " << report.alm_outer_iterations
                        << ", " << report.alm_inner_solves
                        << ", " << report.alm_topology_changes
                        << ", " << report.adaptive_coarse_segments
                        << ", " << report.adaptive_fine_segments
                        << ", " << report.alm_max_violation
                        << ", " << static_cast<int>(report.alm_certified)
                        << ", " << report.alm_warm_start_seconds * 1.0e3
                        << ", " << report.dense_integral_seconds * 1.0e3
                        << ", " << report.control_point_seconds * 1.0e3
                        << ", " << report.hull_transform_seconds * 1.0e3
                        << ", " << report.hull_hodograph_seconds * 1.0e3
                        << ", " << report.hull_position_residual_seconds * 1.0e3
                        << ", " << report.hull_derivative_residual_seconds * 1.0e3
                        << ", " << report.hull_reverse_hodograph_seconds * 1.0e3
                        << ", " << report.hull_backward_add_seconds * 1.0e3
                        << ", " << report.hull_discrete_attractor_seconds * 1.0e3
                        << ", " << report.minco_evaluation_seconds * 1.0e3
                        << ", " << report.optimization_seconds * 1.0e3
                        << ", " << report.dense_share_of_minco_evaluation * 100.0
                        << ", " << report.control_point_share_of_minco_evaluation * 100.0
                        << ", " << report.dense_share_of_optimization * 100.0
                        << ", " << (exp_opt_seconds > 0.0
                                      ? report.optimization_seconds / exp_opt_seconds * 100.0
                                      : 0.0)
                        << ", " << (total_replan_seconds > 0.0
                                      ? exp_opt_seconds / total_replan_seconds * 100.0
                                      : 0.0)
                        << ", " << static_cast<int>(report.fast_stop_satisfied)
                        << ", " << report.fast_stop_candidate_checks
                        << ", " << report.fast_stop_cost_passes
                        << ", " << report.fast_stop_decision_step_passes
                        << ", " << report.fast_stop_penalty_change_passes
                        << ", " << report.fast_stop_physical_time_passes
                        << ", " << report.fast_stop_waypoint_passes
                        << ", " << report.fast_stop_gradient_passes
                        << ", " << report.fast_stop_violation_passes
                        << ", " << report.fast_stop_nonstall_passes
                        << ", " << report.fast_stop_base_rule_passes
                        << ", " << report.fast_stop_guarded_rule_passes
                        << ", " << report.warm_start_status
                        << ", " << static_cast<int>(report.warm_start_attempted)
                        << ", " << static_cast<int>(report.warm_start_accepted)
                        << ", " << static_cast<int>(
                                      report.warm_start_topology_resampled)
                        << ", " << report.warm_start_comparison_evaluations
                        << ", " << report.warm_start_seconds * 1.0e3
                        << ", " << report.warm_start_baseline_cost
                        << ", " << report.warm_start_candidate_cost
                        << ", " << report.warm_start_baseline_gradient
                        << ", " << report.warm_start_candidate_gradient
                        << ", " << report.warm_start_baseline_penalty
                        << ", " << report.warm_start_candidate_penalty
                        << ", " << report.warm_start_max_waypoint_shift
                        << ", " << static_cast<int>(report.continuous_feasible)
                        << ", " << static_cast<int>(report.robustly_certified)
                        << ", " << static_cast<int>(report.has_certified_incumbent)
                        << ", " << report.max_normalized_violation
                        << ", " << report.min_position_margin
                        << ", " << report.primal_residual
                        << ", " << report.dual_residual
                        << ", " << report.complementarity_residual
                        << ", " << report.stationarity_residual
                        << ", " << static_cast<int>(report.phase2_triggered)
                        << ", " << report.phase2_packed_constraints
                        << ", " << static_cast<int>(report.jerk_certificate_enabled);
        }
        write_time_ << endl;
    }

    std::pair<std::size_t, int> Fsm::appendLatestReplanLog() {
        LogOneReplan log = planner_ptr_->getLatestReplanLog();
        std::lock_guard<std::mutex> lock(replan_logs_mutex_);
        if (useTrackingLogStream()) {
            tracking_replan_logs_.push_back(std::move(log));
            return {tracking_replan_logs_.size() - 1U, tracking_replan_logs_.back().getRetCode()};
        }
        replan_logs_.push_back(std::move(log));
        return {replan_logs_.size() - 1U, replan_logs_.back().getRetCode()};
    }

    vector<LogOneReplan> Fsm::snapshotReplanLogs() const {
        std::lock_guard<std::mutex> lock(replan_logs_mutex_);
        return replan_logs_;
    }

    vector<LogOneReplan> Fsm::snapshotTrackingReplanLogs() const {
        std::lock_guard<std::mutex> lock(replan_logs_mutex_);
        return tracking_replan_logs_;
    }

    void Fsm::openDiagnosticLogFile(const std::string &path) {
        if (!cfg_.diagnostic_log_en) {
            return;
        }
        boost::system::error_code ec;
        const boost::filesystem::path log_path(path);
        const boost::filesystem::path parent = log_path.parent_path();
        if (!parent.empty()) {
            boost::filesystem::create_directories(parent, ec);
        }
        diagnostic_event_log_.open(path, std::ios::out | std::ios::trunc);
        if (diagnostic_event_log_.is_open()) {
            writeDiagnosticCsvHeader(diagnostic_event_log_);
        }
    }

    void Fsm::openTrackingDiagnosticLogFile(const std::string &path) {
        if (!cfg_.diagnostic_log_en) {
            return;
        }
        boost::system::error_code ec;
        const boost::filesystem::path log_path(path);
        const boost::filesystem::path parent = log_path.parent_path();
        if (!parent.empty()) {
            boost::filesystem::create_directories(parent, ec);
        }
        tracking_diagnostic_event_log_.open(path, std::ios::out | std::ios::trunc);
        if (tracking_diagnostic_event_log_.is_open()) {
            writeDiagnosticCsvHeader(tracking_diagnostic_event_log_);
        }
    }

    vector<Fsm::DiagnosticEvent> Fsm::snapshotDiagnosticEvents() const {
        std::lock_guard<std::mutex> lock(diagnostic_events_mutex_);
        return diagnostic_events_;
    }

    vector<Fsm::DiagnosticEvent> Fsm::snapshotTrackingDiagnosticEvents() const {
        std::lock_guard<std::mutex> lock(diagnostic_events_mutex_);
        return tracking_diagnostic_events_;
    }

    std::string Fsm::diagnosticEventToString(const DiagnosticEvent &event) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "stamp=" << event.stamp
            << ";relative_time=" << event.stamp - system_start_time_
            << ";replan_id=" << event.replan_id
            << ";replan_log_id=" << event.replan_log_id
            << ";level=" << event.level
            << ";event=" << event.event
            << ";task_mode=" << event.task_mode
            << ";machine_state=" << event.machine_state
            << ";detail=" << event.detail
            << ";goal=[" << event.goal_p.x() << "," << event.goal_p.y() << "," << event.goal_p.z() << "]"
            << ";goal_yaw=" << event.goal_yaw
            << ";robot=[" << event.robot_p.x() << "," << event.robot_p.y() << "," << event.robot_p.z() << "]"
            << ";robot_v=[" << event.robot_v.x() << "," << event.robot_v.y() << "," << event.robot_v.z() << "]"
            << ";robot_state_received=" << static_cast<int>(event.robot_state_received)
            << ";traj_seq=" << event.traj_seq
            << ";ret_code=" << event.ret_code
            << ";ret_code_name=" << retCodeName(event.ret_code)
            << ";total_replan_time_ms=" << event.total_replan_time_ms
            << ";remaining_traj_s=" << event.remaining_traj_s
            << ";on_backup=" << static_cast<int>(event.on_backup);
        return oss.str();
    }

    void Fsm::recordDiagnosticEvent(const std::string &level,
                                    const std::string &event,
                                    const std::string &detail,
                                    const int ret_code,
                                    const int traj_seq,
                                    const bool on_backup,
                                    const int replan_log_id,
                                    const uint64_t replan_id_override) {
        if (!cfg_.diagnostic_log_en) {
            return;
        }
        DiagnosticEvent diagnostic_event;
        diagnostic_event.stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
        diagnostic_event.replan_id = replan_id_override == std::numeric_limits<uint64_t>::max()
                                     ? active_replan_id_
                                     : replan_id_override;
        diagnostic_event.replan_log_id = replan_log_id;
        diagnostic_event.level = level;
        diagnostic_event.event = event;
        diagnostic_event.task_mode = cfg_.task_mode_str;
        diagnostic_event.machine_state = MACHINE_STATE_STR[static_cast<int>(machine_state_)];
        diagnostic_event.detail = detail;
        diagnostic_event.goal_p = gi_.goal_p;
        diagnostic_event.goal_yaw = gi_.goal_yaw;
        diagnostic_event.traj_seq = traj_seq;
        diagnostic_event.ret_code = ret_code;
        diagnostic_event.on_backup = on_backup;
        if (planner_ptr_) {
            rog_map::RobotState current_robot_state{};
            planner_ptr_->getRobotState(current_robot_state);
            diagnostic_event.robot_state_received = current_robot_state.rcv;
            if (current_robot_state.rcv) {
                diagnostic_event.robot_p = current_robot_state.p;
                diagnostic_event.robot_v = current_robot_state.v;
            }
            diagnostic_event.total_replan_time_ms = planner_ptr_->getLatestTotalReplanTime() * 1000.0;
            diagnostic_event.remaining_traj_s = planner_ptr_->getCommittedTrajectoryRemainingDuration();
        } else {
            diagnostic_event.robot_state_received = robot_state_.rcv;
            if (robot_state_.rcv) {
                diagnostic_event.robot_p = robot_state_.p;
                diagnostic_event.robot_v = robot_state_.v;
            }
        }

        const bool tracking_log_stream = useTrackingLogStream();
        {
            std::lock_guard<std::mutex> lock(diagnostic_events_mutex_);
            if (tracking_log_stream) {
                tracking_diagnostic_events_.push_back(diagnostic_event);
                if (tracking_diagnostic_event_log_.is_open()) {
                    writeDiagnosticCsvRow(tracking_diagnostic_event_log_, diagnostic_event, system_start_time_);
                    tracking_diagnostic_event_log_.flush();
                }
            } else {
                diagnostic_events_.push_back(diagnostic_event);
                if (diagnostic_event_log_.is_open()) {
                    writeDiagnosticCsvRow(diagnostic_event_log_, diagnostic_event, system_start_time_);
                    diagnostic_event_log_.flush();
                }
            }
        }

        publishDiagnosticEvent(diagnostic_event);
    }

    void Fsm::recordTrackingReplanContext(const std::string &event,
                                          const int ret_code,
                                          const bool prediction_static,
                                          const std::size_t input_prediction_size,
                                          const int replan_log_id) {
        if (!planner_ptr_) {
            return;
        }
        const auto diag = planner_ptr_->getLatestTrackingDiagnosticSnapshot();
        recordDiagnosticEvent(ret_code == FAILED || ret_code == OPT_FAILED ? "WARN" : "INFO",
                              event,
                              fmt::format("ret={};phase={};reason={};input_prediction.size()={};prediction_static={};guide_path.size()={};sfc.size()={};problem.target_prediction.size()={};out_traj_duration={:.3f};keep_old_count={};reject_count={};runtime_manager_enabled={};has_committed_tracking={};committed_remaining={:.3f};runtime_reset={};runtime_preserved={};runtime_reason={};last_commit_wt={:.3f};last_commit_reject_reason={};last_commit_reject_detail={}",
                                          retCodeName(ret_code),
                                          diag.phase,
                                          diag.reason,
                                          input_prediction_size,
                                          static_cast<int>(prediction_static),
                                          diag.guide_path_size,
                                          diag.sfc_size,
                                          diag.target_prediction_size,
                                          diag.out_traj_duration,
                                          diag.consecutive_keep_old,
                                          diag.consecutive_reject,
                                          static_cast<int>(diag.runtime_manager_enabled),
                                          static_cast<int>(diag.has_committed_tracking),
                                          diag.committed_remaining,
                                          static_cast<int>(diag.runtime_reset),
                                          static_cast<int>(diag.runtime_preserved),
                                          diag.runtime_reason.empty()
                                              ? "none"
                                              : diag.runtime_reason,
                                          diag.last_commit_wt,
                                          diag.last_commit_reject_reason.empty()
                                              ? "none"
                                              : diag.last_commit_reject_reason,
                                          diag.last_commit_reject_detail.empty()
                                              ? "none"
                                              : diag.last_commit_reject_detail),
                              ret_code,
                              -1,
                              false,
                              replan_log_id);
    }

    void Fsm::recordTrackingTargetInput(const std::string &source,
                                        const traj_opt::DynamicTargetStates &prediction,
                                        const double source_stamp,
                                        const std::size_t raw_sample_count) {
        if (!cfg_.diagnostic_log_en || !useTrackingLogStream() || prediction.empty()) {
            return;
        }

        const double now = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
        const auto &first = prediction.front();
        const auto &last = prediction.back();
        const bool prediction_static = trackingPredictionStatic(prediction);
        const uint64_t target_seq = ++tracking_target_input_seq_;

        const bool first_log = last_tracking_target_log_time_ < 0.0;
        const bool source_changed = source != last_tracking_target_log_source_;
        const bool size_changed = prediction.size() != last_tracking_target_log_size_;
        const bool static_changed = prediction_static != last_tracking_target_log_static_;
        const bool endpoint_changed =
                (first.position - last_tracking_target_log_front_).norm() > 0.10 ||
                (last.position - last_tracking_target_log_back_).norm() > 0.10;
        const bool period_elapsed = first_log || now - last_tracking_target_log_time_ >= 0.20;
        if (!period_elapsed && !source_changed && !size_changed && !static_changed && !endpoint_changed) {
            return;
        }

        last_tracking_target_log_time_ = now;
        last_tracking_target_log_source_ = source;
        last_tracking_target_log_size_ = prediction.size();
        last_tracking_target_log_front_ = first.position;
        last_tracking_target_log_back_ = last.position;
        last_tracking_target_log_static_ = prediction_static;

        const double prediction_duration = std::max(0.0, last.t - first.t);
        const double source_age = source_stamp > 0.0 ? now - source_stamp : -1.0;
        const double displacement = (last.position - first.position).norm();
        recordDiagnosticEvent("INFO",
                              "tracking_target_input",
                              fmt::format("source={};target_seq={};raw_samples={};prediction.size()={};"
                                          "prediction_duration={:.3f};source_stamp={:.6f};source_age={:.3f};"
                                          "prediction_static={};displacement={:.3f};"
                                          "p0=({:.3f},{:.3f},{:.3f});v0=({:.3f},{:.3f},{:.3f});"
                                          "a0=({:.3f},{:.3f},{:.3f});yaw0={:.3f};yaw_rate0={:.3f};"
                                          "pend=({:.3f},{:.3f},{:.3f});vend=({:.3f},{:.3f},{:.3f});"
                                          "aend=({:.3f},{:.3f},{:.3f});yaw_end={:.3f};yaw_rate_end={:.3f}",
                                          source,
                                          target_seq,
                                          raw_sample_count,
                                          prediction.size(),
                                          prediction_duration,
                                          source_stamp,
                                          source_age,
                                          static_cast<int>(prediction_static),
                                          displacement,
                                          first.position.x(), first.position.y(), first.position.z(),
                                          first.velocity.x(), first.velocity.y(), first.velocity.z(),
                                          first.acceleration.x(), first.acceleration.y(), first.acceleration.z(),
                                          first.yaw,
                                          first.yaw_rate,
                                          last.position.x(), last.position.y(), last.position.z(),
                                          last.velocity.x(), last.velocity.y(), last.velocity.z(),
                                          last.acceleration.x(), last.acceleration.y(), last.acceleration.z(),
                                          last.yaw,
                                          last.yaw_rate));
    }

    void Fsm::saveDiagnosticLogToFile(const std::string &name) {
        if (!cfg_.diagnostic_log_en) {
            return;
        }
        const auto diagnostic_events = snapshotDiagnosticEvents();
        const std::string csv_path = name.empty()
                                     ? LOG_FILE_DIR("diagnostic_events/" +
                                                    BinaryFileHandler<int>::getCurrentTimeStr() + ".csv")
                                     : LOG_FILE_DIR("diagnostic_events/" + name + ".csv");
        boost::system::error_code ec;
        const boost::filesystem::path log_path(csv_path);
        const boost::filesystem::path parent = log_path.parent_path();
        if (!parent.empty()) {
            boost::filesystem::create_directories(parent, ec);
        }
        std::ofstream csv_writer(csv_path, std::ios::out | std::ios::trunc);
        if (!csv_writer.is_open()) {
            return;
        }
        writeDiagnosticCsvHeader(csv_writer);
        for (const auto &event: diagnostic_events) {
            writeDiagnosticCsvRow(csv_writer, event, system_start_time_);
        }
    }

    void Fsm::saveTrackingDiagnosticLogToFile(const std::string &name) {
        if (!cfg_.diagnostic_log_en) {
            return;
        }
        const auto diagnostic_events = snapshotTrackingDiagnosticEvents();
        if (diagnostic_events.empty()) {
            return;
        }
        const std::string csv_path = name.empty()
                                     ? LOG_FILE_DIR("tracking_diagnostic_events/" +
                                                    BinaryFileHandler<int>::getCurrentTimeStr() + ".csv")
                                     : LOG_FILE_DIR("tracking_diagnostic_events/" + name + ".csv");
        boost::system::error_code ec;
        const boost::filesystem::path log_path(csv_path);
        const boost::filesystem::path parent = log_path.parent_path();
        if (!parent.empty()) {
            boost::filesystem::create_directories(parent, ec);
        }
        std::ofstream csv_writer(csv_path, std::ios::out | std::ios::trunc);
        if (!csv_writer.is_open()) {
            return;
        }
        writeDiagnosticCsvHeader(csv_writer);
        for (const auto &event: diagnostic_events) {
            writeDiagnosticCsvRow(csv_writer, event, system_start_time_);
        }
    }

    bool Fsm::ensureLogParentDirectory(const std::string &path) const {
        boost::system::error_code ec;
        const boost::filesystem::path log_path(path);
        const boost::filesystem::path parent = log_path.parent_path();
        if (parent.empty()) {
            return true;
        }
        if (boost::filesystem::exists(parent, ec)) {
            return boost::filesystem::is_directory(parent, ec);
        }
        return boost::filesystem::create_directories(parent, ec);
    }

    bool Fsm::useTrackingLogStream() const {
        return trackingMode() || trackingPerchingMode();
    }

    void Fsm::callReplanOnce() {
        // Replanning runs at a much lower rate than the main FSM timer.  If
        // this callback loses a try-lock race, dropping the entire tick can
        // starve rolling replanning and make the committed backup suffix run
        // to completion.  Wait for the short FSM critical section instead;
        // the expensive state-to-state planner call is still executed without
        // this lock below.
        std::unique_lock<std::mutex> tick_lock(fsm_tick_mutex_);
        if (stop) {
            return;
        }

        TaskExecutor &executor = taskExecutor();
        if (!executor.replanAllowed(*this)) {
            return;
        }

        if (finish_plan) {
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        const bool release_fsm_lock_during_replan = state2stateMode();
        if (release_fsm_lock_during_replan &&
            state2state_replan_in_progress_.exchange(true)) {
            return;
        }
        struct State2StateReplanGuard {
            std::atomic<bool> &in_progress;
            bool active;
            ~State2StateReplanGuard() {
                if (active) {
                    in_progress.store(false);
                }
            }
        } replan_guard{state2state_replan_in_progress_,
                       release_fsm_lock_during_replan};

        TimeConsuming replan_once_time("replan_once_time", false);
        active_replan_id_ = next_replan_id_++;
        const bool perception_replan_trigger = perception_replan_requested_;
        const bool perception_replan_emergency = perception_replan_emergency_;
        const auto perception_report = perception_replan_report_;
        recordDiagnosticEvent("INFO",
                              "replan_start",
                              fmt::format("plugin={};mission={};task={};backend={};phase={};new_goal={};perception_trigger={};perception_ttc={:.3f};perception_reason={}",
                                          executor.name(),
                                          general_planner::architecture::toString(executor.missionMode()),
                                          general_planner::architecture::toString(executor.taskType()),
                                          general_planner::architecture::toString(cfg_.backend_type),
                                          general_planner::architecture::toString(executionPhase()),
                                          static_cast<int>(gi_.new_goal),
                                          static_cast<int>(perception_replan_trigger),
                                          perception_replan_trigger ? perception_report.time_to_collision : -1.0,
                                          perception_replan_trigger ? perception_report.reason : "none"));

        PlanRequest replan_request;
        replan_request.identity = executor.identity(*this);
        replan_request.invocation = general_planner::architecture::PlanInvocation::REPLAN;
        replan_request.phase = executionPhase();
        replan_request.new_goal = gi_.new_goal;
        replan_request.new_task = task_new_;
        replan_request.perception_trigger = perception_replan_trigger;
        replan_request.perception_emergency = perception_replan_emergency;

        // The state-to-state backend already serializes planner calls with
        // GeneralPlanner::replan_lock_. Holding fsm_tick_mutex_ here made a
        // slow or stuck frontend search freeze the whole FSM, including the
        // transition performed when the currently committed trajectory ends.
        // Release only the FSM lock around the expensive planner call and
        // reacquire it before applying the result to the state machine.
        if (release_fsm_lock_during_replan) {
            tick_lock.unlock();
        }
        PlanResult plan_result = executor.replan(*this, replan_request);
        if (release_fsm_lock_during_replan) {
            tick_lock.lock();
        }
        const TaskPlanContext &replan_context = plan_result.context;
        const RET_CODE ret_code = static_cast<RET_CODE>(plan_result.ret_code);
        if (replan_context.missing_input || replan_context.handled) {
            return;
        }
        const bool replan_tracking_static = replan_context.tracking_prediction_static;
        const std::size_t replan_tracking_input_prediction_size =
                replan_context.tracking_input_prediction_size;
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }
        recordDiagnosticEvent(ret_code == FAILED ? "WARN" : "INFO",
                              "replan_result",
                              fmt::format("new_goal={};ret={}", static_cast<int>(gi_.new_goal),
                                          retCodeName(ret_code)),
                              ret_code);
        if (perception_replan_trigger) {
            recordDiagnosticEvent(ret_code == FAILED ? "WARN" : "INFO",
                                  "perception_replan_result",
                                  fmt::format("ret={};emergency={};ttc={:.3f};reason={};grid={};pos=({:.3f},{:.3f},{:.3f})",
                                              retCodeName(ret_code),
                                              static_cast<int>(perception_replan_emergency),
                                              perception_report.time_to_collision,
                                              perception_report.reason,
                                              gridTypeName(perception_report.grid_type),
                                              perception_report.collision_pos.x(),
                                              perception_report.collision_pos.y(),
                                              perception_report.collision_pos.z()),
                                  ret_code);
        }

        const auto commit_decision = commit_governor_.decideReplan(
                general_planner::architecture::CommitGovernorInput{
                        plan_result,
                        true,
                        explorationMode()});
        recordDiagnosticEvent("INFO",
                              "commit_governor_decision",
                              fmt::format("invocation=replan;action={};next_phase={};reason={};mission_node={};detail={}",
                                          general_planner::architecture::toString(commit_decision.action),
                                          general_planner::architecture::toString(commit_decision.next_phase),
                                          commit_decision.reason,
                                          replan_context.mission_node.empty()
                                              ? "none"
                                              : replan_context.mission_node,
                                          replan_context.decision_detail.empty()
                                              ? "none"
                                              : replan_context.decision_detail),
                              ret_code);

        switch (commit_decision.action) {
            case general_planner::architecture::CommitAction::EMERGENCY_STOP:
                ChangeState("ReplanTimerCallback", EMER_STOP);
                break;
            case general_planner::architecture::CommitAction::KEEP_OLD_TRAJECTORY:
                if (ret_code == FAILED && executor.trackingLike()) {
                    recordDiagnosticEvent("WARN",
                                          "tracking_replan_failed_keep_current",
                                          fmt::format("prediction_static={};input_prediction.size()={}",
                                                      static_cast<int>(replan_tracking_static),
                                                      replan_tracking_input_prediction_size),
                                          ret_code);
                } else if (ret_code == FAILED &&
                           executor.goalLike() &&
                           perception_replan_trigger &&
                           perception_replan_emergency) {
                    recordDiagnosticEvent("WARN",
                                          "perception_replan_failed_keep_current",
                                          fmt::format("keep_current_trajectory=1;ttc={:.3f};reason={}",
                                                      perception_report.time_to_collision,
                                                      perception_report.reason),
                                          ret_code);
                }
                if (commit_decision.publish_trajectory) {
                    publishPolyTraj();
                    if (replan_tracking_static) {
                        ChangeState("ReplanTimerCallback", STATIC_TRACKING);
                    } else if (machine_state_ != FOLLOW_TRAJ) {
                        ChangeState("ReplanTimerCallback", FOLLOW_TRAJ);
                    }
                }
                break;
            case general_planner::architecture::CommitAction::RETRY_PLANNING:
                ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
                break;
            case general_planner::architecture::CommitAction::FINISH_MISSION:
                gi_.new_goal = false;
                task_new_ = false;
                finish_plan = true;
                cout << GREEN << " -- [Fsm] Exploration finished." << RESET << endl;
                ChangeState("ReplanTimerCallback", WAIT_GOAL);
                break;
            case general_planner::architecture::CommitAction::COMMIT_CANDIDATE:
                if (commit_decision.clear_goal) {
                    gi_.new_goal = false;
                }
                if (commit_decision.clear_task_new) {
                    task_new_ = false;
                }
                if (commit_decision.publish_trajectory) {
                    publishPolyTraj();
                }
                if (executor.trackingLike()) {
                    if (replan_tracking_static) {
                        ChangeState("ReplanTimerCallback", STATIC_TRACKING);
                    } else if (machine_state_ != FOLLOW_TRAJ) {
                        ChangeState("ReplanTimerCallback", FOLLOW_TRAJ);
                    }
                }
                break;
            case general_planner::architecture::CommitAction::HOLD:
                ChangeState("ReplanTimerCallback", HOLD_TRACKING);
                break;
            case general_planner::architecture::CommitAction::BRAKE:
            case general_planner::architecture::CommitAction::REQUEST_NEW_INPUT:
            case general_planner::architecture::CommitAction::NOOP:
            default:
                break;
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        const auto log_id = appendLatestReplanLog();
        const bool replan_warn = log_id.second == FAILED || log_id.second == OPT_FAILED;
        recordDiagnosticEvent(replan_warn ? "WARN" : "INFO",
                              "replan_log_saved",
                              fmt::format("ret={}", retCodeName(log_id.second)),
                              log_id.second,
                              -1,
                              false,
                              static_cast<int>(log_id.first));
        if (executor.trackingLike()) {
            recordTrackingReplanContext("tracking_replan_context",
                                        log_id.second,
                                        replan_tracking_static,
                                        replan_tracking_input_prediction_size,
                                        static_cast<int>(log_id.first));
        }
        if (perception_replan_trigger) {
            perception_replan_requested_ = false;
            perception_replan_emergency_ = false;
        }
        WriteTimeToLog();
    }

    void Fsm::callPerceptionSafetyCheckOnce() {
        if (!(cfg_.perception_replan_check_en || cfg_.dynamic_obstacle_layer_enable) ||
            !(state2stateMode() || se3AggressiveMode())) {
            return;
        }

        bool should_trigger_replan = false;
        {
            std::unique_lock<std::mutex> tick_lock(fsm_tick_mutex_, std::try_to_lock);
            if (!tick_lock.owns_lock()) {
                return;
            }
            if (stop || machine_state_ != FOLLOW_TRAJ || finish_plan || plan_from_rest_) {
                return;
            }

            planner_ptr_->getRobotState(robot_state_);
            const double now = ros_ptr_->getSimTime();
            if (!robot_state_.rcv || (now - robot_state_.rcv_time) > 0.2) {
                return;
            }

            general_planner::GeneralPlanner::CommittedTrajectorySafetyReport report;
            const bool safe = planner_ptr_->checkCommittedPositionTrajectorySafety(
                    cfg_.perception_replan_check_horizon,
                    cfg_.perception_replan_check_dt,
                    cfg_.perception_replan_consecutive_hits,
                    cfg_.perception_replan_unknown_as_occupied,
                    &report);
            if (safe) {
                if (perception_replan_requested_) {
                    perception_replan_requested_ = false;
                    perception_replan_emergency_ = false;
                }
                return;
            }

            if (!report.valid) {
                return;
            }

            const bool emergency =
                    report.time_to_collision <= std::max(0.0, cfg_.perception_replan_emergency_horizon);
            const double min_interval = std::max(0.0, cfg_.perception_replan_min_interval);
            const bool interval_ok =
                    last_perception_replan_request_time_ < 0.0 ||
                    now - last_perception_replan_request_time_ >= min_interval ||
                    emergency;
            const general_planner::architecture::SafetyMonitorContext safety_context{
                    activeTaskIdentity(),
                    executionPhase(),
                    "perception_replan"};
            const auto safety_decision = safety_monitor_.evaluateCommittedCollision(
                    safety_context,
                    report.valid,
                    safe,
                    report.time_to_collision,
                    cfg_.perception_replan_emergency_horizon,
                    interval_ok);

            const double log_period = std::max(0.0, cfg_.perception_replan_log_period);
            if (last_perception_replan_log_time_ < 0.0 ||
                now - last_perception_replan_log_time_ >= log_period ||
                emergency) {
                last_perception_replan_log_time_ = now;
                ros_ptr_->warn(" -- [Fsm] Perception safety collision: reason={}, ttc={:.3f}s, grid={}, pos=({:.2f},{:.2f},{:.2f}), emergency={}.",
                               report.reason,
                               report.time_to_collision,
                               gridTypeName(report.grid_type),
                               report.collision_pos.x(),
                               report.collision_pos.y(),
                               report.collision_pos.z(),
                               static_cast<int>(emergency));
                recordDiagnosticEvent("WARN",
                                      "perception_traj_collision",
                                      fmt::format("reason={};ttc={:.3f};grid={};hits={};pos=({:.3f},{:.3f},{:.3f});emergency={}",
                                                  report.reason,
                                                  report.time_to_collision,
                                                  gridTypeName(report.grid_type),
                                                  report.hit_count,
                                                  report.collision_pos.x(),
                                                  report.collision_pos.y(),
                                                  report.collision_pos.z(),
                                                  static_cast<int>(emergency)));
            }

            if (!interval_ok) {
                return;
            }

            perception_replan_requested_ = true;
            perception_replan_emergency_ = safety_decision.emergency;
            perception_replan_report_ = report;
            last_perception_replan_request_time_ = now;
            should_trigger_replan = safety_decision.request_replan;
            recordDiagnosticEvent(general_planner::architecture::toString(safety_decision.severity),
                                  "perception_replan_request",
                                  fmt::format("reason={};action={};ttc={:.3f};grid={};hits={};pos=({:.3f},{:.3f},{:.3f});emergency={}",
                                              report.reason,
                                              general_planner::architecture::toString(safety_decision.action),
                                              report.time_to_collision,
                                              gridTypeName(report.grid_type),
                                              report.hit_count,
                                              report.collision_pos.x(),
                                              report.collision_pos.y(),
                                              report.collision_pos.z(),
                                              static_cast<int>(safety_decision.emergency)));
        }

        if (should_trigger_replan) {
            callReplanOnce();
        }
    }

    void Fsm::callMainFsmOnce() {
        std::unique_lock<std::mutex> tick_lock;
        if (!trackingMode() && !trackingPerchingMode()) {
            tick_lock = std::unique_lock<std::mutex>(fsm_tick_mutex_, std::try_to_lock);
            if (!tick_lock.owns_lock()) {
                return;
            }
        }
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!activeTaskReady()) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                // A state-to-state rolling replan can still be finishing after
                // the command trajectory reaches its end. Do not start a
                // concurrent plan-from-rest on the same planner; keep the FSM
                // callback non-blocking until the rolling replan returns.
                if (state2stateMode() &&
                    state2state_replan_in_progress_.load()) {
                    return;
                }
                active_replan_id_ = next_replan_id_++;
                TaskExecutor &executor = taskExecutor();
                if (executor.goalLike() && closeToGoal(0.1)) {
                    recordDiagnosticEvent("INFO",
                                          "goal_reached_without_plan",
                                          "distance_below_threshold=0.1",
                                          -1,
                                          -1,
                                          false,
                                          -1,
                                          0);
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    state2state_plan_from_rest_fail_count_ = 0;
                    finish_plan = true;
                    return;
                }
                recordDiagnosticEvent("INFO",
                                      "plan_from_rest_start",
                                      fmt::format("plugin={};mission={};task={};backend={};phase={};new_goal={};task_new={}",
                                                  executor.name(),
                                                  general_planner::architecture::toString(executor.missionMode()),
                                                  general_planner::architecture::toString(executor.taskType()),
                                                  general_planner::architecture::toString(cfg_.backend_type),
                                                  general_planner::architecture::toString(executionPhase()),
                                                  static_cast<int>(gi_.new_goal),
                                                  static_cast<int>(task_new_)));
                PlanRequest plan_request;
                plan_request.identity = executor.identity(*this);
                plan_request.invocation = general_planner::architecture::PlanInvocation::PLAN_FROM_REST;
                plan_request.phase = executionPhase();
                plan_request.new_goal = gi_.new_goal;
                plan_request.new_task = task_new_;
                PlanResult plan_result = executor.plan(*this, plan_request);
                const TaskPlanContext &plan_context = plan_result.context;
                const int retcode = plan_result.ret_code;
                if (plan_context.missing_input || plan_context.handled) {
                    return;
                }
                const bool planned_tracking_static = plan_context.tracking_prediction_static;
                const std::size_t plan_tracking_input_prediction_size =
                        plan_context.tracking_input_prediction_size;
                if (executor.goalLike() && !planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    recordDiagnosticEvent("WARN",
                                          "plan_from_rest_result",
                                          "goal_valid=0",
                                          retcode);
                    gi_.new_goal = false;
                    started_ = false;
                    state2state_plan_from_rest_fail_count_ = 0;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (executor.goalLike()) {
                    recordDiagnosticEvent(retcode == FAILED ? "WARN" : "INFO",
                                          "plan_from_rest_result",
                                          fmt::format("ret={};goal_valid={}",
                                                      retCodeName(retcode),
                                                      static_cast<int>(planner_ptr_->goalValid())),
                                          retcode);
                } else if (executor.trackingLike()) {
                    recordDiagnosticEvent(retcode == FAILED || retcode == OPT_FAILED ? "WARN" : "INFO",
                                          "plan_from_rest_result",
                                          fmt::format("ret={};prediction_static={};input_prediction.size()={}",
                                                      retCodeName(retcode),
                                                      static_cast<int>(planned_tracking_static),
                                                      plan_tracking_input_prediction_size),
                                          retcode);
                } else {
                    recordDiagnosticEvent(retcode == FAILED || retcode == OPT_FAILED ? "WARN" : "INFO",
                                          "plan_from_rest_result",
                                          fmt::format("ret={};task={}", retCodeName(retcode), executor.name()),
                                          retcode);
                }

                const auto commit_decision = commit_governor_.decidePlanFromRest(
                        general_planner::architecture::CommitGovernorInput{
                                plan_result,
                                planner_ptr_->goalValid(),
                                explorationMode()});
                recordDiagnosticEvent("INFO",
                                      "commit_governor_decision",
                                      fmt::format("invocation=plan_from_rest;action={};next_phase={};reason={};mission_node={};detail={}",
                                                  general_planner::architecture::toString(commit_decision.action),
                                                  general_planner::architecture::toString(commit_decision.next_phase),
                                                  commit_decision.reason,
                                                  plan_context.mission_node.empty()
                                                      ? "none"
                                                      : plan_context.mission_node,
                                                  plan_context.decision_detail.empty()
                                                      ? "none"
                                                      : plan_context.decision_detail),
                                      retcode);

                switch (commit_decision.action) {
                    case general_planner::architecture::CommitAction::KEEP_OLD_TRAJECTORY:
                        if (executor.trackingLike()) {
                            resetTrackingPlanFromRestFailureState();
                        }
                        plan_from_rest_ = true;
                        finish_plan = false;
                        if (commit_decision.publish_trajectory) {
                            publishPolyTraj();
                        }
                        ChangeState("MainFsmCallback",
                                    planned_tracking_static ? STATIC_TRACKING : FOLLOW_TRAJ);
                        break;
                    case general_planner::architecture::CommitAction::FINISH_MISSION:
                        gi_.new_goal = false;
                        task_new_ = false;
                        plan_from_rest_ = false;
                        finish_plan = true;
                        cout << GREEN << " -- [Fsm] Exploration finished." << RESET << endl;
                        ChangeState("MainFsmCallback", WAIT_GOAL);
                        break;
                    case general_planner::architecture::CommitAction::COMMIT_CANDIDATE:
                        if (commit_decision.clear_goal) {
                            gi_.new_goal = false;
                        }
                        if (commit_decision.clear_task_new) {
                            task_new_ = false;
                        }
                        if (explorationMode()) {
                            exploration_plan_from_rest_fail_count_ = 0;
                        } else if (executor.goalLike()) {
                            state2state_plan_from_rest_fail_count_ = 0;
                        } else if (executor.trackingLike()) {
                            resetTrackingPlanFromRestFailureState();
                        }
                        plan_from_rest_ = true;
                        finish_plan = commit_decision.finish_plan;
                        if (commit_decision.publish_trajectory) {
                            publishPolyTraj();
                        }
                        ChangeState("MainFsmCallback",
                                    executor.trackingLike() && planned_tracking_static
                                        ? STATIC_TRACKING
                                        : FOLLOW_TRAJ);
                        break;
                    case general_planner::architecture::CommitAction::EMERGENCY_STOP:
                        ChangeState("MainFsmCallback", EMER_STOP);
                        break;
                    case general_planner::architecture::CommitAction::REQUEST_NEW_INPUT:
                        gi_.new_goal = false;
                        started_ = false;
                        plan_from_rest_ = false;
                        finish_plan = true;
                        state2state_plan_from_rest_fail_count_ = 0;
                        ChangeState("MainFsmCallback", WAIT_GOAL);
                        break;
                    case general_planner::architecture::CommitAction::RETRY_PLANNING:
                    default:
                        if (explorationMode()) {
                            ++exploration_plan_from_rest_fail_count_;
                            cout << YELLOW << " -- [Fsm] Exploration PlanFromRest failed, keep retrying. consecutive_failures="
                                 << exploration_plan_from_rest_fail_count_ << RESET << endl;
                            if (task_new_) {
                                task_new_ = false;
                                recordDiagnosticEvent("INFO",
                                                      "exploration_initial_task_reset_consumed",
                                                      "retry_after_first_plan_from_rest_attempt",
                                                      retcode);
                            }
                            started_ = true;
                            finish_plan = false;
                            recordDiagnosticEvent("WARN",
                                                  "exploration_plan_from_rest_consecutive_failure",
                                                  fmt::format("count={};ret={}",
                                                              exploration_plan_from_rest_fail_count_,
                                                              retCodeName(retcode)),
                                                  retcode);
                        } else if (executor.goalLike()) {
                            ++state2state_plan_from_rest_fail_count_;
                            const int failure_limit = cfg_.state2state_plan_from_rest_max_failures;
                            cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan. consecutive_failures="
                                 << state2state_plan_from_rest_fail_count_;
                            if (failure_limit > 0) {
                                cout << "/" << failure_limit;
                            }
                            cout << RESET << endl;
                            recordDiagnosticEvent("WARN",
                                                  "plan_from_rest_consecutive_failure",
                                                  fmt::format("count={};limit={};clear_goal_on_limit={}",
                                                              state2state_plan_from_rest_fail_count_,
                                                              failure_limit,
                                                              static_cast<int>(
                                                                      cfg_.state2state_clear_goal_on_plan_failure)),
                                                  retcode);
                            if (failure_limit > 0 &&
                                state2state_plan_from_rest_fail_count_ >= failure_limit) {
                                recordDiagnosticEvent("ERROR",
                                                      "plan_from_rest_failure_limit_reached",
                                                      fmt::format("count={};clear_goal={}",
                                                                  state2state_plan_from_rest_fail_count_,
                                                                  static_cast<int>(
                                                                          cfg_.state2state_clear_goal_on_plan_failure)),
                                                      retcode);
                                if (cfg_.state2state_clear_goal_on_plan_failure) {
                                    cout << YELLOW << " -- [Fsm] PlanFromRest failed "
                                         << state2state_plan_from_rest_fail_count_
                                         << " times, clear current state2state goal and wait for a new goal."
                                         << RESET << endl;
                                    gi_.new_goal = false;
                                    started_ = false;
                                    plan_from_rest_ = false;
                                    finish_plan = true;
                                    ChangeState("PlanFromRestFailureLimit", WAIT_GOAL);
                                }
                                state2state_plan_from_rest_fail_count_ = 0;
                            }
                        } else if (executor.trackingLike()) {
                            handleTrackingPlanFromRestFailure(retcode,
                                                              planned_tracking_static,
                                                              plan_tracking_input_prediction_size);
                        } else {
                            cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                        }
                        break;
                }
                const auto log_id = appendLatestReplanLog();
                const bool replan_warn = log_id.second == FAILED || log_id.second == OPT_FAILED;
                recordDiagnosticEvent(replan_warn ? "WARN" : "INFO",
                                      "replan_log_saved",
                                      fmt::format("ret={}", retCodeName(log_id.second)),
                                      log_id.second,
                                      -1,
                                      false,
                                      static_cast<int>(log_id.first));
                if (executor.trackingLike()) {
                    recordTrackingReplanContext("tracking_plan_from_rest_context",
                                                log_id.second,
                                                planned_tracking_static,
                                                plan_tracking_input_prediction_size,
                                                static_cast<int>(log_id.first));
                }
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                break;
            }
            case STATIC_TRACKING:
            case HOLD_TRACKING: {
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    bool Fsm::state2stateMode() const {
        return cfg_.task_mode == TaskMode::STATE_TO_STATE;
    }

    bool Fsm::trackingMode() const {
        return cfg_.task_mode == TaskMode::TRACKING;
    }

    bool Fsm::trackingPerchingMode() const {
        return cfg_.task_mode == TaskMode::TRACKING_PERCHING;
    }

    bool Fsm::perchingMode() const {
        return cfg_.task_mode == TaskMode::PERCHING;
    }

    bool Fsm::dynamicTakeoffMode() const {
        return cfg_.task_mode == TaskMode::DYNAMIC_TAKEOFF;
    }

    bool Fsm::explorationMode() const {
        return cfg_.task_mode == TaskMode::EXPLORATION;
    }

    bool Fsm::se3AggressiveMode() const {
        return cfg_.task_type == general_planner::architecture::TaskType::STATE_TO_STATE &&
               cfg_.backend_type == general_planner::architecture::BackendType::SE3;
    }

    bool Fsm::trackingExecutionState() const {
        return machine_state_ == FOLLOW_TRAJ ||
               machine_state_ == STATIC_TRACKING;
    }

    bool Fsm::trackingPerchingPerchingActive() const {
        const bool composite_tracking_perching = trackingPerchingMode();
        return (cfg_.tracking_perching_enable || composite_tracking_perching) &&
               (trackingMode() || composite_tracking_perching) &&
               planner_ptr_ &&
               planner_ptr_->trackingPerchingPerchingActive();
    }

    void Fsm::setTaskModeFromString(const std::string &mode) {
        const auto requested_backend =
                general_planner::architecture::backendTypeFromLegacyMode(mode);
        const std::string normalized = normalizeTaskMode(mode);
        const TaskMode new_mode = taskModeFromString(normalized);
        const auto refresh_task_semantics = [this, &requested_backend]() {
            cfg_.task_type = taskTypeFromTaskMode(cfg_.task_mode);
            cfg_.mission_mode = missionModeFromTaskMode(cfg_.task_mode);
            if (requested_backend.has_value()) {
                cfg_.backend_type = *requested_backend;
                cfg_.planning_backend_str = general_planner::architecture::toString(cfg_.backend_type);
                return;
            }
            const bool state2state_with_tracking_backend =
                    cfg_.task_type == general_planner::architecture::TaskType::STATE_TO_STATE &&
                    (cfg_.backend_type == general_planner::architecture::BackendType::JERK_TRACKING ||
                     cfg_.backend_type == general_planner::architecture::BackendType::SNAP_TRACKING);
            const bool tracking_with_state_backend =
                    cfg_.task_type == general_planner::architecture::TaskType::TRACKING &&
                    cfg_.backend_type == general_planner::architecture::BackendType::SE3;
            if (cfg_.backend_type == general_planner::architecture::BackendType::AUTO ||
                state2state_with_tracking_backend ||
                tracking_with_state_backend) {
                cfg_.backend_type = general_planner::architecture::defaultBackendForTask(cfg_.task_type);
                cfg_.planning_backend_str = general_planner::architecture::toString(cfg_.backend_type);
            }
        };
        const bool backend_change_only =
                requested_backend.has_value() && *requested_backend != cfg_.backend_type;
        if (new_mode == cfg_.task_mode && !backend_change_only) {
            if (new_mode == TaskMode::EXPLORATION) {
                finish_plan = false;
                task_new_ = true;
                started_ = true;
                exploration_plan_from_rest_fail_count_ = 0;
            }
            if (new_mode == TaskMode::TRACKING_PERCHING && planner_ptr_) {
                planner_ptr_->setTrackingPerchingRequest(true);
                finish_plan = false;
                task_new_ = true;
                started_ = true;
            }
            return;
        }
        cout << YELLOW << " -- [Fsm] Task mode switch: " << cfg_.task_mode_str
             << " -> " << normalized;
        if (requested_backend.has_value()) {
            cout << " backend=" << general_planner::architecture::toString(*requested_backend);
        }
        cout << RESET << endl;
        cfg_.task_mode_str = normalized;
        cfg_.task_mode = new_mode;
        refresh_task_semantics();
        resetTaskExecutor();
        finish_plan = false;
        task_new_ = true;
        exploration_plan_from_rest_fail_count_ = 0;
        if (new_mode == TaskMode::DYNAMIC_TAKEOFF) {
            perching_surface_first_rcv_time_ = -1.0;
            last_dynamic_takeoff_wait_log_time_ = -1.0;
        }
        if (new_mode == TaskMode::EXPLORATION ||
            new_mode == TaskMode::TRACKING_PERCHING) {
            started_ = true;
        }
        if (planner_ptr_) {
            planner_ptr_->setTrackingPerchingRequest(new_mode == TaskMode::TRACKING_PERCHING);
        }
        perching_contact_reached_ = false;
    }

    bool Fsm::trackingTaskReady() {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (tracking_target_prediction_.empty() || tracking_target_rcv_time_ < 0.0) {
            return false;
        }
        return (ros_ptr_->getSimTime() - tracking_target_rcv_time_) <= cfg_.task_timeout;
    }

    bool Fsm::perchingTaskReady() {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (perching_surface_rcv_time_ < 0.0) {
            return false;
        }
        return (ros_ptr_->getSimTime() - perching_surface_rcv_time_) <= cfg_.task_timeout;
    }

    bool Fsm::dynamicTakeoffTaskReady() {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (perching_surface_rcv_time_ < 0.0 ||
            (ros_ptr_->getSimTime() - perching_surface_rcv_time_) > cfg_.task_timeout) {
            return false;
        }
        const double start_delay = std::max(0.0, cfg_.dynamic_takeoff_start_delay);
        if (start_delay <= 1.0e-6) {
            return true;
        }
        if (perching_surface_first_rcv_time_ < 0.0) {
            return false;
        }
        const double elapsed = ros_ptr_->getSimTime() - perching_surface_first_rcv_time_;
        if (elapsed + 1.0e-6 < start_delay) {
            const double now = ros_ptr_->getSimTime();
            if (last_dynamic_takeoff_wait_log_time_ < 0.0 ||
                now - last_dynamic_takeoff_wait_log_time_ > 1.0) {
                last_dynamic_takeoff_wait_log_time_ = now;
                cout << YELLOW << " -- [Fsm] Dynamic takeoff waits on-platform contact motion: "
                     << std::fixed << std::setprecision(2) << elapsed << " / "
                     << start_delay << " s" << RESET << endl;
            }
            return false;
        }
        return true;
    }

    void Fsm::resetTrackingPlanFromRestFailureState() {
        tracking_plan_from_rest_fail_count_ = 0;
        tracking_plan_from_rest_backoff_until_ = -1.0;
        last_tracking_plan_from_rest_backoff_log_time_ = -1.0;
    }

    bool Fsm::trackingPlanFromRestBackoffActive() {
        if (tracking_plan_from_rest_backoff_until_ < 0.0) {
            return false;
        }
        const double now = ros_ptr_->getSimTime();
        if (now + 1.0e-6 >= tracking_plan_from_rest_backoff_until_) {
            tracking_plan_from_rest_backoff_until_ = -1.0;
            last_tracking_plan_from_rest_backoff_log_time_ = -1.0;
            return false;
        }

        if (last_tracking_plan_from_rest_backoff_log_time_ < 0.0 ||
            now - last_tracking_plan_from_rest_backoff_log_time_ >= 1.0) {
            last_tracking_plan_from_rest_backoff_log_time_ = now;
            recordDiagnosticEvent("INFO",
                                  "tracking_plan_from_rest_backoff_wait",
                                  fmt::format("remaining={:.3f};consecutive_failures={}",
                                              tracking_plan_from_rest_backoff_until_ - now,
                                              tracking_plan_from_rest_fail_count_));
        }
        return true;
    }

    void Fsm::handleTrackingPlanFromRestFailure(const int retcode,
                                                const bool prediction_static,
                                                const std::size_t input_prediction_size) {
        ++tracking_plan_from_rest_fail_count_;

        const auto diag = planner_ptr_->getLatestTrackingDiagnosticSnapshot();
        const int failure_limit = cfg_.tracking_plan_from_rest_max_failures;
        const bool failure_limit_reached =
                failure_limit > 0 && tracking_plan_from_rest_fail_count_ >= failure_limit;
        const bool no_committed_tracking =
                !diag.has_committed_tracking || diag.committed_remaining <= 1.0e-3;
        const bool static_no_committed_limit =
                prediction_static && no_committed_tracking && failure_limit_reached;

        const double backoff = 0.0;
        tracking_plan_from_rest_backoff_until_ = -1.0;
        last_tracking_plan_from_rest_backoff_log_time_ = -1.0;

        cout << YELLOW << " -- [Fsm] Tracking PlanFromRest failed, retry from GENERATE_TRAJ. consecutive_failures="
             << tracking_plan_from_rest_fail_count_;
        if (failure_limit > 0) {
            cout << "/" << failure_limit;
        }
        cout << RESET << endl;

        recordDiagnosticEvent(static_no_committed_limit ? "ERROR" : "WARN",
                              static_no_committed_limit
                                  ? "tracking_plan_from_rest_failure_limit_reached"
                                  : "tracking_plan_from_rest_consecutive_failure",
                              fmt::format("count={};limit={};prediction_static={};input_prediction.size()={};"
                                          "has_committed_tracking={};committed_remaining={:.3f};"
                                          "backoff={:.3f};static_finish_on_limit={};"
                                          "diag_phase={};diag_reason={}",
                                          tracking_plan_from_rest_fail_count_,
                                          failure_limit,
                                          static_cast<int>(prediction_static),
                                          input_prediction_size,
                                          static_cast<int>(diag.has_committed_tracking),
                                          diag.committed_remaining,
                                          backoff,
                                          static_cast<int>(cfg_.tracking_static_finish_on_plan_failure),
                                          diag.phase,
                                          diag.reason),
                              retcode);
    }

    bool Fsm::activeTaskReady() {
        return taskExecutor().ready(*this);
    }

    bool Fsm::shouldGenerateAfterTrajFinish() {
        return taskExecutor().shouldGenerateAfterTrajFinish(*this);
    }

    general_planner::architecture::ExecutionPhase Fsm::executionPhase() const {
        using general_planner::architecture::ExecutionPhase;
        switch (machine_state_) {
            case INIT:
            case WAIT_GOAL:
            case YAWING:
                return ExecutionPhase::WAITING_INPUT;
            case GENERATE_TRAJ:
                return ExecutionPhase::PLANNING;
            case FOLLOW_TRAJ:
            case STATIC_TRACKING:
                return ExecutionPhase::EXECUTING;
            case HOLD_TRACKING:
                return ExecutionPhase::HOLDING;
            case EMER_STOP:
                return ExecutionPhase::EMERGENCY;
            default:
                return ExecutionPhase::WAITING_INPUT;
        }
    }

    general_planner::architecture::TaskIdentity Fsm::activeTaskIdentity() {
        if (mission_orchestrator_.snapshot().mission == general_planner::architecture::MissionMode::IDLE) {
            return taskExecutor().identity(*this);
        }
        return mission_orchestrator_.activeIdentity();
    }

    const general_planner::architecture::MissionSnapshot &Fsm::missionSnapshot() const {
        return mission_orchestrator_.snapshot();
    }

    bool Fsm::markTrackingFinishedIfStaticTarget() {
        return false;
    }

    Fsm::ExecutedTrajectoryFinishResult Fsm::handleExecutedTrajectoryFinished(
            const std::string &source,
            const int trajectory_id,
            const int trajectory_seq,
            const bool on_backup,
            const bool record_regular_finish,
            const bool mark_static_target_finished) {
        ExecutedTrajectoryFinishResult result;
        const bool close_to_goal = closeToGoal(0.1);
        const bool tracking_unfinished =
                (trackingMode() || trackingPerchingMode()) &&
                !trackingPerchingPerchingActive() &&
                !close_to_goal;
        const bool log_finish_once =
                !tracking_unfinished ||
                last_tracking_unfinished_traj_seq_ != trajectory_seq;

        if ((!record_regular_finish && !tracking_unfinished) ||
            (record_regular_finish && log_finish_once)) {
            cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
        }
        if (record_regular_finish && log_finish_once) {
            recordDiagnosticEvent("INFO",
                                  "trajectory_finished",
                                  fmt::format("trajectory_id={};close_to_goal={}",
                                              trajectory_id,
                                              static_cast<int>(close_to_goal)),
                                  -1,
                                  trajectory_seq,
                                  on_backup);
        }

        const bool tracking_perching_contact = trackingPerchingPerchingActive();
        if (!tracking_unfinished && (perchingMode() || tracking_perching_contact)) {
            {
                std::lock_guard<std::mutex> lock(task_mutex_);
                perching_contact_reached_ = true;
                perching_contact_surface_position_ = perching_surface_.position;
                task_new_ = false;
            }
            gi_.new_goal = false;
            if (tracking_perching_contact) {
                planner_ptr_->markTrackingPerchingContact();
            }
            cout << GREEN << " -- [Perching] PERCHING_CONTACT" << RESET << endl;
        }

        if (tracking_unfinished) {
            task_new_ = true;
            plan_from_rest_ = true;
            finish_plan = false;
            if (log_finish_once) {
                last_tracking_unfinished_traj_seq_ = trajectory_seq;
                const std::string hold_reason =
                        source == "getPoseFromTraj"
                            ? "tracking pose query reached trajectory end before target"
                            : "tracking trajectory finished before reaching target";
                const bool hold_committed =
                        planner_ptr_->commitTrackingHoldTrajectory(hold_reason);
                recordDiagnosticEvent("WARN",
                                      "tracking_trajectory_finished_reacquire",
                                      fmt::format("trajectory_id={};hold_committed={}",
                                                  trajectory_id,
                                                  static_cast<int>(hold_committed)),
                                      -1,
                                      trajectory_seq,
                                      on_backup);
                if (hold_committed) {
                    publishPolyTraj();
                }
            }
            result.tracking_unfinished = true;
            return result;
        }

        if (mark_static_target_finished) {
            markTrackingFinishedIfStaticTarget();
        }
        if (shouldGenerateAfterTrajFinish()) {
            ChangeState(source, GENERATE_TRAJ);
        } else {
            ChangeState(source, WAIT_GOAL);
        }
        result.state_changed = true;
        return result;
    }

    void Fsm::logStaticTrackingReplanDecision(const std::string &reason) {
        const double now = ros_ptr_->getSimTime();
        const double log_period = std::max(0.0, cfg_.tracking_static_replan_log_period);
        if (last_static_tracking_replan_log_time_ >= 0.0 &&
            now - last_static_tracking_replan_log_time_ < log_period) {
            return;
        }
        last_static_tracking_replan_log_time_ = now;
        ros_ptr_->info(" -- [Fsm] Static tracking replan decision: {}", reason);
    }

    bool Fsm::trackingCommittedTrajectoryUnsafe() const {
        const auto map_manager = planner_ptr_->getMapManager();
        if (map_manager == nullptr || !map_manager->ready()) {
            return false;
        }

        const Trajectory traj = planner_ptr_->getCommittedPositionTrajectory();
        if (traj.empty()) {
            return true;
        }

        const double remaining = planner_ptr_->getCommittedTrajectoryRemainingDuration();
        if (remaining <= 1.0e-3) {
            return false;
        }

        const double current_t = std::clamp(ros_ptr_->getSimTime() - traj.start_WT,
                                            0.0,
                                            traj.getTotalDuration());
        const double horizon = std::min(remaining, std::max(0.0, cfg_.tracking_static_safety_check_horizon));
        const double dt = std::max(0.05, cfg_.tracking_static_safety_check_dt);

        Vec3f last = traj.getPos(current_t);
        if (!last.allFinite()) {
            return true;
        }

        for (double offset = 0.0; offset <= horizon + 1.0e-6; offset += dt) {
            const double t = std::min(traj.getTotalDuration(), current_t + offset);
            const Vec3f pos = traj.getPos(t);
            if (!pos.allFinite() || !map_manager->insideLocalMap(pos)) {
                return true;
            }
            const GridType grid_type = map_manager->getInfGridType(pos);
            if (grid_type == GridType::OCCUPIED || grid_type == GridType::OUT_OF_MAP) {
                return true;
            }
            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager->isLineFree(last, pos, true, false)) {
                return true;
            }
            last = pos;
        }
        return false;
    }

    traj_opt::DynamicTargetStates Fsm::filterStaticTrackingPrediction(
            const traj_opt::DynamicTargetStates &prediction) const {
        if (prediction.empty()) {
            return prediction;
        }

        traj_opt::DynamicTargetStates filtered = prediction;
        const Vec3f ref = filtered.front().position;
        double max_span = 0.0;
        double max_vel = 0.0;
        for (const auto &state : filtered) {
            max_span = std::max(max_span, (state.position - ref).norm());
            max_vel = std::max(max_vel, state.velocity.norm());
        }

        const double pos_eps = std::max(0.0, cfg_.tracking_static_position_epsilon);
        const double vel_eps = std::max(0.0, cfg_.tracking_static_prediction_filter_velocity_epsilon);
        if (max_span > pos_eps && max_vel > vel_eps) {
            return filtered;
        }

        const double static_yaw = filtered.front().yaw;
        for (auto &state : filtered) {
            state.position = ref;
            state.velocity.setZero();
            state.acceleration.setZero();
            state.yaw = static_yaw;
            state.yaw_rate = 0.0;
        }
        return filtered;
    }

    bool Fsm::trackingPredictionChanged(const traj_opt::DynamicTargetStates &a,
                                        const traj_opt::DynamicTargetStates &b) const {
        if (a.empty() || b.empty()) {
            return true;
        }

        const auto &a_front = a.front();
        const auto &b_front = b.front();
        const auto &a_back = a.back();
        const auto &b_back = b.back();
        const double pos_eps = std::max(0.0, cfg_.tracking_static_position_epsilon);
        const double vel_eps = std::max(0.0, cfg_.tracking_static_velocity_epsilon);
        const double yaw_eps = std::max(0.0, cfg_.tracking_static_yaw_epsilon);

        const bool both_static = trackingPredictionStatic(a) && trackingPredictionStatic(b);
        const double task_pos_eps = both_static
                                        ? std::max(pos_eps, cfg_.tracking_static_task_position_epsilon)
                                        : pos_eps;
        const double task_vel_eps = both_static
                                        ? std::max(vel_eps, cfg_.tracking_static_task_velocity_epsilon)
                                        : vel_eps;
        const auto yawChanged = [yaw_eps](double lhs, double rhs) {
            return std::abs(yawDiff(lhs, rhs)) > yaw_eps;
        };

        return (a_front.position - b_front.position).norm() > task_pos_eps ||
               (a_back.position - b_back.position).norm() > task_pos_eps ||
               (a_front.velocity - b_front.velocity).norm() > task_vel_eps ||
               (a_back.velocity - b_back.velocity).norm() > task_vel_eps ||
               yawChanged(a_front.yaw, b_front.yaw) ||
               yawChanged(a_back.yaw, b_back.yaw);
    }

    bool Fsm::trackingPredictionStatic(const traj_opt::DynamicTargetStates &prediction) const {
        if (prediction.empty()) {
            return false;
        }

        const double pos_eps = std::max(0.0, cfg_.tracking_static_position_epsilon);
        const double vel_eps = std::max(0.0, cfg_.tracking_static_velocity_epsilon);
        const Vec3f ref = prediction.front().position;
        for (const auto &state : prediction) {
            if ((state.position - ref).norm() > pos_eps ||
                state.velocity.norm() > vel_eps) {
                return false;
            }
        }
        return true;
    }

    bool Fsm::shouldSkipStaticTrackingReplan(const traj_opt::DynamicTargetStates &prediction) {
        if (!trackingPredictionStatic(prediction)) {
            logStaticTrackingReplanDecision("target prediction is moving");
            return false;
        }
        if (task_new_) {
            logStaticTrackingReplanDecision("task_new is true; target moved outside static hold noise band");
            return false;
        }
        const double remaining = planner_ptr_->getCommittedTrajectoryRemainingDuration();
        const double replan_time = std::max(0.0, cfg_.tracking_static_replan_remaining_time);
        if (remaining <= replan_time) {
            logStaticTrackingReplanDecision(
                    fmt::format("trajectory ending soon: remaining={:.3f}s <= {:.3f}s", remaining, replan_time));
            return false;
        }
        if (trackingCommittedTrajectoryUnsafe()) {
            logStaticTrackingReplanDecision("committed trajectory has safety risk");
            return false;
        }
        logStaticTrackingReplanDecision(
                fmt::format("skip static hold replan: remaining={:.3f}s, target static, trajectory safe", remaining));
        return true;
    }

    void Fsm::setTrackingTargetPrediction(const traj_opt::DynamicTargetStates &prediction) {
        if (prediction.empty()) {
            return;
        }
        const traj_opt::DynamicTargetStates filtered_prediction = filterStaticTrackingPrediction(prediction);
        bool changed = true;
        bool reacquired_after_timeout = false;
        double stale_duration = 0.0;
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            const double now = ros_ptr_->getSimTime();
            const bool had_previous_prediction =
                    !tracking_target_prediction_.empty() && tracking_target_rcv_time_ >= 0.0;
            stale_duration = had_previous_prediction ? now - tracking_target_rcv_time_ : 0.0;
            reacquired_after_timeout =
                    had_previous_prediction && stale_duration > std::max(0.0, cfg_.task_timeout);
            changed = reacquired_after_timeout ||
                      trackingPredictionChanged(tracking_target_prediction_, filtered_prediction);
            tracking_target_prediction_ = filtered_prediction;
            tracking_target_rcv_time_ = now;
            task_new_ = task_new_ || changed;
        }
        gi_.goal_p = filtered_prediction.back().position;
        gi_.goal_yaw = filtered_prediction.back().yaw;
        gi_.new_goal = gi_.new_goal || changed;
        if (changed) {
            finish_plan = false;
            resetTrackingPlanFromRestFailureState();
        }
        started_ = true;
        if (reacquired_after_timeout) {
            finish_plan = false;
            recordDiagnosticEvent("INFO",
                                  "tracking_target_reacquired",
                                  fmt::format("stale_duration={:.3f};timeout={:.3f};prediction.size()={};force_task_new=1",
                                              stale_duration,
                                              cfg_.task_timeout,
                                              filtered_prediction.size()));
        }
    }

    void Fsm::setPerchingSurface(const traj_opt::PerchingSurfaceState &surface) {
        bool changed = true;
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            const double now = ros_ptr_->getSimTime();
            if (perching_surface_rcv_time_ >= 0.0) {
                const double dt = std::max(0.0, now - perching_surface_rcv_time_);
                const Vec3f predicted_position =
                        perching_surface_.position +
                        perching_surface_.velocity * dt +
                        0.5 * perching_surface_.acceleration * dt * dt;
                const Vec3f predicted_velocity =
                        perching_surface_.velocity + perching_surface_.acceleration * dt;
                const double predicted_yaw = perching_surface_.yaw + perching_surface_.yaw_rate * dt;
                const double position_error = (surface.position - predicted_position).norm();
                const double velocity_error = (surface.velocity - predicted_velocity).norm();
                const double yaw_error = std::abs(yawDiff(surface.yaw, predicted_yaw));
                const double yaw_rate_error = std::abs(surface.yaw_rate - perching_surface_.yaw_rate);
                const Vec3f new_z = surface.surface_z.norm() > 1.0e-6
                                         ? surface.surface_z.normalized()
                                         : Vec3f::UnitZ();
                const Vec3f old_z = perching_surface_.surface_z.norm() > 1.0e-6
                                         ? perching_surface_.surface_z.normalized()
                                         : Vec3f::UnitZ();
                const double normal_error = (new_z - old_z).norm();
                changed = position_error > 0.35 ||
                          velocity_error > 0.35 ||
                          yaw_error > 0.35 ||
                          yaw_rate_error > 0.35 ||
                          normal_error > 0.35;
            }
            if (perching_contact_reached_) {
                const double unlock_dist = 0.35;
                if ((surface.position - perching_contact_surface_position_).norm() <= unlock_dist) {
                    changed = false;
                } else {
                    perching_contact_reached_ = false;
                    changed = true;
                }
            }
            perching_surface_ = surface;
            perching_surface_rcv_time_ = now;
            if (perching_surface_first_rcv_time_ < 0.0) {
                perching_surface_first_rcv_time_ = now;
            }
            task_new_ = task_new_ || changed;
        }
        gi_.goal_p = surface.position;
        gi_.goal_yaw = surface.yaw;
        gi_.new_goal = gi_.new_goal || changed;
        started_ = true;
    }

    bool Fsm::getTrackingTargetPrediction(traj_opt::DynamicTargetStates &prediction) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (tracking_target_prediction_.empty() || tracking_target_rcv_time_ < 0.0 ||
            (ros_ptr_->getSimTime() - tracking_target_rcv_time_) > cfg_.task_timeout) {
            return false;
        }
        prediction = tracking_target_prediction_;
        return true;
    }

    bool Fsm::getPerchingSurface(traj_opt::PerchingSurfaceState &surface) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (perching_surface_rcv_time_ < 0.0 ||
            (ros_ptr_->getSimTime() - perching_surface_rcv_time_) > cfg_.task_timeout) {
            return false;
        }
        surface = perching_surface_;
        return true;
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p,
                                const Quatf &q,
                                const GoalHeightMode height_mode) {

        if (!p.allFinite()) {
            recordDiagnosticEvent("WARN",
                                  "goal_rejected",
                                  "reason=non_finite_position",
                                  -1,
                                  -1,
                                  false,
                                  -1,
                                  0);
            return;
        }
        if (!checker::quaternionValidOrDisabled(q)) {
            recordDiagnosticEvent("WARN",
                                  "goal_rejected",
                                  "reason=invalid_quaternion",
                                  -1,
                                  -1,
                                  false,
                                  -1,
                                  0);
            return;
        }

        auto click_point = p;
        click_point.z() = resolveGoalHeight(p.z(), cfg_.click_height, height_mode);
        recordDiagnosticEvent("INFO",
                              "goal_received",
                              fmt::format("raw=[{:.3f},{:.3f},{:.3f}];click_height={:.3f};adjusted=[{:.3f},{:.3f},{:.3f}];height_source={}",
                                          p.x(), p.y(), p.z(),
                                          cfg_.click_height,
                                          click_point.x(), click_point.y(), click_point.z(),
                                          goalHeightSourceName(cfg_.click_height, height_mode)),
                              -1,
                              -1,
                              false,
                              -1,
                              0);
        const bool had_goal = started_;
        const Vec3f last_goal_p = gi_.goal_p;
        const double last_goal_yaw = gi_.goal_yaw;

        gi_.goal_p = click_point;
        if (planner_ptr_->getMapManager()->getInfGridType(click_point) == GridType::OCCUPIED) {
            if (planner_ptr_->getMapManager()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)) {
                cout << GREEN << " -- [Fsm] Project occupied goal to " << RESET << gi_.goal_p.transpose() << endl;
            } else {
                fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
                recordDiagnosticEvent("WARN",
                                      "goal_rejected",
                                      fmt::format("reason=occupied_or_no_free_projection;raw=[{:.3f},{:.3f},{:.3f}];adjusted=[{:.3f},{:.3f},{:.3f}]",
                                                  p.x(), p.y(), p.z(),
                                                  click_point.x(), click_point.y(), click_point.z()),
                                      -1,
                                      -1,
                                      false,
                                      -1,
                                      0);
                return;
            }
        } else {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        }
        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            recordDiagnosticEvent("INFO",
                                  "goal_rejected",
                                  fmt::format("reason=too_close;distance={:.3f}",
                                              (robot_state_.p - gi_.goal_p).norm()),
                                  -1,
                                  -1,
                                  false,
                                  -1,
                                  0);
            return;
        }

        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        const bool same_yaw = (std::isnan(last_goal_yaw) && std::isnan(gi_.goal_yaw)) ||
                              (std::isfinite(last_goal_yaw) && std::isfinite(gi_.goal_yaw) &&
                               std::fabs(last_goal_yaw - gi_.goal_yaw) < 0.02);
        if (had_goal && (last_goal_p - gi_.goal_p).norm() < 0.05 && same_yaw) {
            recordDiagnosticEvent("INFO",
                                  "goal_duplicated",
                                  fmt::format("position_delta={:.3f};same_yaw={}",
                                              (last_goal_p - gi_.goal_p).norm(),
                                              static_cast<int>(same_yaw)),
                                  -1,
                                  -1,
                                  false,
                                  -1,
                                  0);
            return;
        }

        started_ = true;
        gi_.new_goal = true;
        state2state_plan_from_rest_fail_count_ = 0;
        recordDiagnosticEvent("INFO",
                              "goal_accepted",
                              fmt::format("projected=[{:.3f},{:.3f},{:.3f}];projection_distance={:.3f};goal_yaw={:.3f};had_goal={}",
                                          gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z(),
                                          (click_point - gi_.goal_p).norm(),
                                          gi_.goal_yaw,
                                          static_cast<int>(had_goal)),
                              -1,
                              -1,
                              false,
                              -1,
                              0);
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        recordDiagnosticEvent("INFO",
                              "fsm_state_transition",
                              fmt::format("caller={};from={};to={}",
                                          call_func,
                                          MACHINE_STATE_STR[int(machine_state_)],
                                          MACHINE_STATE_STR[int(new_state)]));
        machine_state_ = new_state;
        mission_orchestrator_.setExecutionPhase(executionPhase(), call_func);
    }
}
