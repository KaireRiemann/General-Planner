#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Px4State.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/TrakingPerformance.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "ommpc_controller.hpp"
#include "ommpc_controller/fsm_changeConfig.h"

namespace
{
double clampValue(const double value, const double low, const double high)
{
  return std::max(low, std::min(high, value));
}

Eigen::Vector3d rpyFromQuaternion(const Eigen::Quaterniond &q)
{
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::asin(clampValue(-R(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}
}  // namespace

enum ExecTrajState
{
  HOVER = 10,
  POLY_TRAJ = 11,
  POS_CMD_TRAJ = 12,
  TAKEOFF = 13,
  LAND = 14
};

class OMMPC_EXAMPLE
{
private:
  ros::Publisher cmd_pub_;
  ros::Publisher traj_start_trigger_pub_;
  ros::Publisher px4_state_pub_;
  ros::Publisher tracking_perf_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber extended_state_sub_;
  ros::Subscriber mpc_traj_sub_;
  ros::Subscriber pos_cmd_sub_;
  ros::Subscriber takeoff_land_sub_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_srv_;
  ros::Timer exec_timer_;

  mavros_msgs::State state_;
  mavros_msgs::State state_before_offboard_;
  mavros_msgs::ExtendedState extended_state_;
  Odom_Data_t odom_data_;
  Imu_Data_t imu_data_;
  quadrotor_msgs::PositionCommand latest_pos_cmd_;
  ros::Time latest_pos_cmd_time_{0.0};

  ExecTrajState exec_traj_state_{HOVER};
  Parameter_t param_;
  Trajectory_Data_t trajectory_data_;
  MpcController ommpc_controller_;

  bool command_mode_enabled_{true};
  bool takeoff_enabled_{false};
  bool last_takeoff_enabled_{false};
  bool takeoff_trigger_{false};
  bool land_enabled_{false};
  bool last_land_enabled_{false};
  bool land_trigger_{false};
  bool hover_pose_initialized_{false};
  bool pending_start_trigger_{false};
  uint64_t command_seq_{0};

  ros::Time takeoff_land_start_time_{0.0};
  ros::Time pending_start_trigger_time_{0.0};
  ros::Time land_detect_start_time_{0.0};
  Eigen::Vector4d hover_pose_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d takeoff_land_start_pose_{Eigen::Vector4d::Zero()};

  bool enu_frame_{true};
  bool vel_in_body_{false};
  bool auto_start_command_{true};
  bool use_pos_cmd_fallback_{true};
  bool enable_thrust_estimation_{true};
  bool auto_takeoff_land_enable_{true};
  bool auto_arm_enable_{true};
  bool auto_trigger_after_takeoff_{true};
  double ctrl_freq_{100.0};
  double odom_timeout_{0.5};
  double imu_timeout_{0.5};
  double pos_cmd_timeout_{0.2};
  double takeoff_spinup_time_{2.0};
  double takeoff_trigger_delay_{1.0};
  double takeoff_reach_tol_{0.08};
  double max_takeoff_velocity_{0.2};
  double land_descent_height_{3.0};
  double land_detect_velocity_{0.12};
  double land_detect_keep_time_{0.8};
  double min_thrust_signal_{0.04};
  double max_thrust_signal_{0.90};

  int line_cnt_{0};
  int number_of_steps_{0};
  std::vector<std::vector<double>> test_trajectory_;
  std::vector<Eigen::Vector3d> quad_positions_;
  std::vector<Eigen::Vector3d> quad_velocities_;
  std::vector<double> yaws_;

  dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig> state_change_server_;
  dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig>::CallbackType state_change_cb_type_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odom_data_.feed(msg, enu_frame_, vel_in_body_);
    if (!hover_pose_initialized_)
    {
      setHoverWithOdom();
    }
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    imu_data_.feed(msg, enu_frame_);
  }

  void stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    state_ = *msg;
  }

  void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg)
  {
    extended_state_ = *msg;
  }

  void trajectoryCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg)
  {
    trajectory_data_.feed_from_quadrotor_msgs(msg);
  }

  void positionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
  {
    latest_pos_cmd_ = *msg;
    latest_pos_cmd_time_ = ros::Time::now();
  }

  void takeoffLandCallback(const quadrotor_msgs::TakeoffLandConstPtr &msg)
  {
    if (msg->takeoff_hight > 0.05 && msg->takeoff_hight < 5.0)
    {
      param_.takeoff_height = msg->takeoff_hight;
    }

    if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)
    {
      takeoff_trigger_ = true;
      land_trigger_ = false;
      ROS_INFO("[MPCctrl] Received TAKEOFF command.");
    }
    else if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
    {
      land_trigger_ = true;
      takeoff_trigger_ = false;
      ROS_INFO("[MPCctrl] Received LAND command.");
    }
  }

  void stateChangeCallback(ommpc_controller::fsm_changeConfig &config, uint32_t)
  {
    last_takeoff_enabled_ = takeoff_enabled_;
    last_land_enabled_ = land_enabled_;
    command_mode_enabled_ = config.command_or_hover;
    takeoff_enabled_ = config.takeoff_enabled;
    land_enabled_ = config.land_enabled;
    if (!last_takeoff_enabled_ && takeoff_enabled_)
    {
      takeoff_trigger_ = true;
    }
    if (!last_land_enabled_ && land_enabled_)
    {
      land_trigger_ = true;
    }
  }

  std::string stateName() const
  {
    switch (exec_traj_state_)
    {
      case HOVER:
        return "HOVER";
      case POLY_TRAJ:
        return "POLY_TRAJ";
      case POS_CMD_TRAJ:
        return "POS_CMD_TRAJ";
      case TAKEOFF:
        return "TAKEOFF";
      case LAND:
        return "LAND";
      default:
        return "UNKNOWN";
    }
  }

  bool odomFresh(const ros::Time &now) const
  {
    return odom_data_.recv_new_msg && (now - odom_data_.rcv_stamp).toSec() < odom_timeout_;
  }

  bool imuFresh(const ros::Time &now) const
  {
    return imu_data_.recv_new_msg && (now - imu_data_.rcv_stamp).toSec() < imu_timeout_;
  }

  bool flightReady() const
  {
    return state_.armed && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD;
  }

  bool posCmdValid(const ros::Time &now) const
  {
    return use_pos_cmd_fallback_ &&
           !latest_pos_cmd_time_.isZero() &&
           (now - latest_pos_cmd_time_).toSec() < pos_cmd_timeout_ &&
           latest_pos_cmd_.trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTROY_STATUS_ABORT &&
           latest_pos_cmd_.trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
  }

  bool polyTrajValid(const ros::Time &now) const
  {
    return trajectory_data_.exec_traj == 1 &&
           !trajectory_data_.traj_queue.empty() &&
           now <= trajectory_data_.total_traj_end_time + ros::Duration(0.2);
  }

  double getYawFromQuaternion(const Eigen::Quaterniond &q) const
  {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

  void setHoverWithOdom()
  {
    hover_pose_.head<3>() = odom_data_.p;
    hover_pose_(3) = getYawFromQuaternion(odom_data_.q);
    hover_pose_initialized_ = true;
  }

  void setTakeoffLandStartWithOdom()
  {
    takeoff_land_start_pose_.head<3>() = odom_data_.p;
    takeoff_land_start_pose_(3) = getYawFromQuaternion(odom_data_.q);
    takeoff_land_start_time_ = ros::Time::now();
  }

  bool toggleOffboardMode(const bool on)
  {
    mavros_msgs::SetMode mode_cmd;
    if (on)
    {
      state_before_offboard_ = state_;
      if (state_before_offboard_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
      {
        state_before_offboard_.mode = "MANUAL";
      }
      mode_cmd.request.custom_mode = mavros_msgs::State::MODE_PX4_OFFBOARD;
    }
    else
    {
      mode_cmd.request.custom_mode = state_before_offboard_.mode.empty() ? "MANUAL" : state_before_offboard_.mode;
    }

    if (!(set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent))
    {
      ROS_ERROR("[MPCctrl] %s OFFBOARD rejected by PX4.", on ? "Enter" : "Exit");
      return false;
    }
    return true;
  }

  bool toggleArmDisarm(const bool arm)
  {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    if (!(arming_client_srv_.call(arm_cmd) && arm_cmd.response.success))
    {
      ROS_ERROR("[MPCctrl] %s rejected by PX4.", arm ? "ARM" : "DISARM");
      return false;
    }
    return true;
  }

  void publishStartTrigger()
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose = odom_data_.msg.pose.pose;
    traj_start_trigger_pub_.publish(msg);
    ROS_INFO("[MPCctrl] Published /traj_start_trigger.");
  }

  void publishPx4State()
  {
    quadrotor_msgs::Px4State msg;
    msg.seq = command_seq_;
    if (!state_.armed)
    {
      msg.px4_state = 1;
    }
    else if (exec_traj_state_ == TAKEOFF)
    {
      msg.px4_state = 2;
    }
    else if (exec_traj_state_ == LAND)
    {
      msg.px4_state = 5;
    }
    else if (exec_traj_state_ == POLY_TRAJ || exec_traj_state_ == POS_CMD_TRAJ)
    {
      msg.px4_state = 4;
    }
    else
    {
      msg.px4_state = 3;
    }
    px4_state_pub_.publish(msg);
  }

  void sendCommand(const Controller_Output_t &output)
  {
    mavros_msgs::AttitudeTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.body_rate.x = std::isfinite(output.bodyrates(0)) ? output.bodyrates(0) : 0.0;
    cmd.body_rate.y = std::isfinite(output.bodyrates(1)) ? output.bodyrates(1) : 0.0;
    cmd.body_rate.z = std::isfinite(output.bodyrates(2)) ? output.bodyrates(2) : 0.0;
    const double thrust = std::isfinite(output.thrust) ? output.thrust : min_thrust_signal_;
    cmd.thrust = clampValue(thrust, min_thrust_signal_, max_thrust_signal_);
    cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    cmd_pub_.publish(cmd);
    ++command_seq_;
  }

  Controller_Output_t idleOutput() const
  {
    Controller_Output_t output;
    output.bodyrates.setZero();
    output.thrust = min_thrust_signal_;
    return output;
  }

  void fillStateMsg(quadrotor_msgs::QuadrotorState &state_msg,
                    const Eigen::Vector3d &p,
                    const Eigen::Vector3d &v,
                    const Eigen::Vector3d &a,
                    const Eigen::Vector3d &j,
                    const Eigen::Vector3d &rpy,
                    const Eigen::Vector3d &bodyrates,
                    const double thrust) const
  {
    state_msg.thrust = thrust;
    state_msg.velocity_norm = v.norm();
    state_msg.acceleration_norm = a.norm();
    state_msg.jerk_norm = j.norm();
    state_msg.position.x = p.x();
    state_msg.position.y = p.y();
    state_msg.position.z = p.z();
    state_msg.velocity.x = v.x();
    state_msg.velocity.y = v.y();
    state_msg.velocity.z = v.z();
    state_msg.acceleration.x = a.x();
    state_msg.acceleration.y = a.y();
    state_msg.acceleration.z = a.z();
    state_msg.jerk.x = j.x();
    state_msg.jerk.y = j.y();
    state_msg.jerk.z = j.z();
    state_msg.attitude.x = rpy.x();
    state_msg.attitude.y = rpy.y();
    state_msg.attitude.z = rpy.z();
    state_msg.angular_velocity.x = bodyrates.x();
    state_msg.angular_velocity.y = bodyrates.y();
    state_msg.angular_velocity.z = bodyrates.z();
  }

  void publishTrackingPerformance(const Controller_Output_t &output)
  {
    if (tracking_perf_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::VectorXd x_des;
    Eigen::VectorXd u_des;
    ommpc_controller_.getDesiredStart(x_des, u_des);
    if (x_des.size() != nstate || u_des.size() != nu)
    {
      return;
    }

    quadrotor_msgs::TrakingPerformance msg;
    msg.header.stamp = ros::Time::now();
    msg.fsm_state_id.data = static_cast<int>(exec_traj_state_);
    msg.fsm_state.data = stateName();
    msg.mpc_solve_time = ommpc_controller_.getLastSolveTime();
    msg.suggest_hover_percentage = ommpc_controller_.getSuggestedHoverPercentage();

    const Eigen::Vector3d ref_p = x_des.head<3>();
    const Eigen::Quaterniond ref_q(x_des(3), x_des(4), x_des(5), x_des(6));
    const Eigen::Vector3d ref_v = x_des.tail<3>();
    const Eigen::Vector3d ref_bodyrates(u_des(1), u_des(2), u_des(3));
    fillStateMsg(msg.reference,
                 ref_p,
                 ref_v,
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero(),
                 rpyFromQuaternion(ref_q),
                 ref_bodyrates,
                 u_des(0));

    fillStateMsg(msg.command,
                 ref_p,
                 ref_v,
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero(),
                 rpyFromQuaternion(ref_q),
                 output.bodyrates,
                 output.thrust);

    const Eigen::Vector3d fb_rpy = rpyFromQuaternion(odom_data_.q);
    fillStateMsg(msg.feedback,
                 odom_data_.p,
                 odom_data_.v,
                 imu_data_.a,
                 Eigen::Vector3d::Zero(),
                 fb_rpy,
                 odom_data_.w,
                 0.0);

    fillStateMsg(msg.error,
                 ref_p - odom_data_.p,
                 ref_v - odom_data_.v,
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero(),
                 rpyFromQuaternion(odom_data_.q.inverse() * ref_q),
                 output.bodyrates - odom_data_.w,
                 output.thrust);
    tracking_perf_pub_.publish(msg);
  }

  bool runHover(Controller_Output_t &u)
  {
    if (!flightReady())
    {
      u = idleOutput();
      return true;
    }
    ommpc_controller_.setHoverReference(hover_pose_);
    return ommpc_controller_.execMPC(odom_data_, u);
  }

  bool runPolyTrajectory(const ros::Time &now, Controller_Output_t &u)
  {
    if (!flightReady())
    {
      exec_traj_state_ = HOVER;
      return runHover(u);
    }
    if (trajectory_data_.traj_queue.empty() || trajectory_data_.exec_traj != 1)
    {
      exec_traj_state_ = HOVER;
      setHoverWithOdom();
      return runHover(u);
    }

    oneTraj_Data_t &traj_info = trajectory_data_.traj_queue.front();
    if (now < traj_info.traj_start_time)
    {
      return runHover(u);
    }
    if (now > traj_info.traj_end_time)
    {
      if (param_.use_trajectory_ending_pos && traj_info.traj.getPieceNum() > 0)
      {
        hover_pose_.head<3>() = traj_info.traj.getJuncPos(traj_info.traj.getPieceNum());
        hover_pose_(3) = getYawFromQuaternion(odom_data_.q);
        hover_pose_initialized_ = true;
      }
      else
      {
        setHoverWithOdom();
      }
      trajectory_data_.exec_traj = 0;
      exec_traj_state_ = HOVER;
      ROS_INFO("[MPCctrl] Polynomial trajectory finished. POLY_TRAJ --> HOVER.");
      return runHover(u);
    }

    const double traj_time = (now - traj_info.traj_start_time).toSec();
    ommpc_controller_.setTrajectoryReference(traj_info.traj,
                                             traj_time,
                                             hover_pose_(3),
                                             traj_info.yaw_traj,
                                             odom_data_);
    return ommpc_controller_.execMPC(odom_data_, u);
  }

  bool buildPositionCommandReference()
  {
    const Eigen::Vector3d p0(latest_pos_cmd_.position.x,
                             latest_pos_cmd_.position.y,
                             latest_pos_cmd_.position.z);
    const Eigen::Vector3d v0(latest_pos_cmd_.velocity.x,
                             latest_pos_cmd_.velocity.y,
                             latest_pos_cmd_.velocity.z);
    const Eigen::Vector3d a0(latest_pos_cmd_.acceleration.x,
                             latest_pos_cmd_.acceleration.y,
                             latest_pos_cmd_.acceleration.z);
    if (!p0.allFinite() || !v0.allFinite() || !a0.allFinite())
    {
      return false;
    }

    for (int i = 0; i <= nstep; ++i)
    {
      const double dt = static_cast<double>(i) * param_.step_T;
      quad_positions_[i] = p0 + v0 * dt + 0.5 * a0 * dt * dt;
      quad_velocities_[i] = v0 + a0 * dt;
      yaws_[i] = latest_pos_cmd_.yaw + latest_pos_cmd_.yaw_dot * dt;
    }
    return true;
  }

  bool runPositionCommandReference(Controller_Output_t &u)
  {
    if (!flightReady() || !buildPositionCommandReference())
    {
      exec_traj_state_ = HOVER;
      return runHover(u);
    }
    const double yaw_now = getYawFromQuaternion(odom_data_.q);
    ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
    return ommpc_controller_.execMPC(odom_data_, u);
  }

  bool runTakeoff(const ros::Time &now, Controller_Output_t &u)
  {
    const double elapsed = (now - takeoff_land_start_time_).toSec();
    if (elapsed < takeoff_spinup_time_)
    {
      u = idleOutput();
      return true;
    }

    const double ramp_elapsed = elapsed - takeoff_spinup_time_;
    for (int i = 0; i <= nstep; ++i)
    {
      const double dt = ramp_elapsed + static_cast<double>(i) * param_.step_T;
      const double dz = std::min(param_.takeoff_land_speed * dt, param_.takeoff_height);
      const bool reached = dz >= param_.takeoff_height - 1.0e-6;
      quad_positions_[i] = takeoff_land_start_pose_.head<3>() + Eigen::Vector3d(0.0, 0.0, dz);
      quad_velocities_[i] = reached ? Eigen::Vector3d::Zero()
                                    : Eigen::Vector3d(0.0, 0.0, param_.takeoff_land_speed);
      yaws_[i] = takeoff_land_start_pose_(3);
    }

    const double yaw_now = getYawFromQuaternion(odom_data_.q);
    ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
    const bool ok = ommpc_controller_.execMPC(odom_data_, u);
    if (odom_data_.p.z() >= takeoff_land_start_pose_(2) + param_.takeoff_height - takeoff_reach_tol_)
    {
      setHoverWithOdom();
      exec_traj_state_ = HOVER;
      pending_start_trigger_ = auto_trigger_after_takeoff_;
      pending_start_trigger_time_ = now + ros::Duration(takeoff_trigger_delay_);
      ROS_INFO("[MPCctrl] TAKEOFF succeeded. TAKEOFF --> HOVER.");
    }
    return ok;
  }

  bool landedDetected(const ros::Time &now, const double ref_z)
  {
    if (extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
      return true;
    }

    const bool low_velocity = odom_data_.v.norm() < land_detect_velocity_;
    const bool command_below_robot = (ref_z - odom_data_.p.z()) < -0.25;
    if (low_velocity && command_below_robot)
    {
      if (land_detect_start_time_.isZero())
      {
        land_detect_start_time_ = now;
      }
      return (now - land_detect_start_time_).toSec() > land_detect_keep_time_;
    }
    land_detect_start_time_ = ros::Time(0);
    return false;
  }

  bool runLand(const ros::Time &now, Controller_Output_t &u)
  {
    const double elapsed = std::max(0.0, (now - takeoff_land_start_time_).toSec());
    double ref_z = takeoff_land_start_pose_(2);
    for (int i = 0; i <= nstep; ++i)
    {
      const double dt = elapsed + static_cast<double>(i) * param_.step_T;
      const double dz = std::min(param_.takeoff_land_speed * dt, land_descent_height_);
      const bool reached = dz >= land_descent_height_ - 1.0e-6;
      quad_positions_[i] = takeoff_land_start_pose_.head<3>() - Eigen::Vector3d(0.0, 0.0, dz);
      quad_velocities_[i] = reached ? Eigen::Vector3d::Zero()
                                    : Eigen::Vector3d(0.0, 0.0, -param_.takeoff_land_speed);
      yaws_[i] = takeoff_land_start_pose_(3);
      if (i == 0)
      {
        ref_z = quad_positions_[i].z();
      }
    }

    const double yaw_now = getYawFromQuaternion(odom_data_.q);
    ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
    const bool ok = ommpc_controller_.execMPC(odom_data_, u);
    if (landedDetected(now, ref_z))
    {
      if (toggleArmDisarm(false))
      {
        exec_traj_state_ = HOVER;
        setHoverWithOdom();
        trajectory_data_.exec_traj = 0;
        u = idleOutput();
        ROS_INFO("[MPCctrl] LAND succeeded. LAND --> HOVER.");
      }
    }
    return ok;
  }

  bool startTakeoff(const ros::Time &now)
  {
    takeoff_trigger_ = false;
    if (!auto_takeoff_land_enable_)
    {
      ROS_ERROR("[MPCctrl] Reject TAKEOFF. auto_takeoff_land is disabled.");
      return false;
    }
    if (!odomFresh(now) || !imuFresh(now))
    {
      ROS_ERROR("[MPCctrl] Reject TAKEOFF. Missing fresh odom/imu.");
      return false;
    }
    if (odom_data_.v.norm() > max_takeoff_velocity_)
    {
      ROS_ERROR("[MPCctrl] Reject TAKEOFF. Odom velocity %.3f m/s is too high.", odom_data_.v.norm());
      return false;
    }
    if (state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD && !toggleOffboardMode(true))
    {
      return false;
    }
    ros::Duration(0.05).sleep();
    ros::spinOnce();
    if (auto_arm_enable_ && !state_.armed && !toggleArmDisarm(true))
    {
      return false;
    }
    setTakeoffLandStartWithOdom();
    trajectory_data_.exec_traj = 0;
    exec_traj_state_ = TAKEOFF;
    ROS_INFO("[MPCctrl] HOVER --> TAKEOFF. height=%.3f speed=%.3f",
             param_.takeoff_height,
             param_.takeoff_land_speed);
    return true;
  }

  bool startLand(const ros::Time &now)
  {
    land_trigger_ = false;
    if (!auto_takeoff_land_enable_)
    {
      ROS_ERROR("[MPCctrl] Reject LAND. auto_takeoff_land is disabled.");
      return false;
    }
    if (!odomFresh(now) || !imuFresh(now))
    {
      ROS_ERROR("[MPCctrl] Reject LAND. Missing fresh odom/imu.");
      return false;
    }
    if (state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD && !toggleOffboardMode(true))
    {
      return false;
    }
    setTakeoffLandStartWithOdom();
    land_detect_start_time_ = ros::Time(0);
    trajectory_data_.exec_traj = 0;
    exec_traj_state_ = LAND;
    ROS_INFO("[MPCctrl] %s --> LAND.", stateName().c_str());
    return true;
  }

  bool readDataFromFile()
  {
    const std::string traj_path = ros::package::getPath("ommpc_controller") + param_.ref_filename;
    std::ifstream file(traj_path.c_str());
    std::string line;
    if (!file.is_open())
    {
      return false;
    }

    while (std::getline(file, line))
    {
      std::istringstream linestream(line);
      std::vector<double> linedata;
      double number;
      while (linestream >> number)
      {
        linedata.push_back(number);
      }
      if (linedata.size() >= 7)
      {
        test_trajectory_.push_back(linedata);
        number_of_steps_++;
      }
    }
    ROS_INFO("[MPCctrl] Loaded txt trajectory %s with %d samples.", traj_path.c_str(), number_of_steps_);
    return number_of_steps_ > 0;
  }

  void getTxtDes()
  {
    for (int cnt = line_cnt_; cnt <= line_cnt_ + nstep; cnt++)
    {
      const int i = cnt - line_cnt_;
      if (nstep + cnt + 1 < number_of_steps_)
      {
        quad_positions_[i] = Eigen::Vector3d(test_trajectory_[line_cnt_ + i][0],
                                             test_trajectory_[line_cnt_ + i][1],
                                             test_trajectory_[line_cnt_ + i][2]);
        quad_velocities_[i] = Eigen::Vector3d(test_trajectory_[line_cnt_ + i][3],
                                              test_trajectory_[line_cnt_ + i][4],
                                              test_trajectory_[line_cnt_ + i][5]);
        yaws_[i] = test_trajectory_[line_cnt_ + i][6];
      }
      else
      {
        quad_positions_[i] = Eigen::Vector3d(test_trajectory_[number_of_steps_ - 1][0],
                                             test_trajectory_[number_of_steps_ - 1][1],
                                             test_trajectory_[number_of_steps_ - 1][2]);
        quad_velocities_[i] = Eigen::Vector3d::Zero();
        yaws_[i] = test_trajectory_[number_of_steps_ - 1][6];
      }
    }
    line_cnt_++;
    if (line_cnt_ > number_of_steps_)
    {
      line_cnt_ = number_of_steps_;
    }
  }

  bool runTxtReference(Controller_Output_t &u)
  {
    if (number_of_steps_ <= 0)
    {
      return false;
    }
    getTxtDes();
    const double yaw_now = getYawFromQuaternion(odom_data_.q);
    ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
    return ommpc_controller_.execMPC(odom_data_, u);
  }

  template <typename T>
  void readParam(const ros::NodeHandle &nh, const std::string &name, T &value)
  {
    nh.param(name, value, value);
  }

  void execFSMCallback(const ros::TimerEvent &)
  {
    exec_timer_.stop();
    if (!ros::ok())
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    Controller_Output_t u = idleOutput();
    bool ret = true;

    if (!odomFresh(now))
    {
      ROS_WARN_THROTTLE(1.0, "[MPCctrl] Waiting for fresh odometry.");
      sendCommand(u);
      exec_timer_.start();
      return;
    }
    if (!imuFresh(now))
    {
      ROS_WARN_THROTTLE(1.0, "[MPCctrl] Waiting for fresh IMU.");
      sendCommand(u);
      exec_timer_.start();
      return;
    }

    if (pending_start_trigger_ && now >= pending_start_trigger_time_)
    {
      pending_start_trigger_ = false;
      command_mode_enabled_ = true;
      publishStartTrigger();
    }

    if (land_trigger_ && exec_traj_state_ != LAND)
    {
      startLand(now);
    }

    switch (exec_traj_state_)
    {
      case HOVER:
      {
        if (takeoff_trigger_)
        {
          startTakeoff(now);
        }

        if (exec_traj_state_ == HOVER)
        {
          if (command_mode_enabled_ && flightReady() && polyTrajValid(now))
          {
            exec_traj_state_ = POLY_TRAJ;
            ret = runPolyTrajectory(now, u);
          }
          else if (command_mode_enabled_ && flightReady() && posCmdValid(now))
          {
            exec_traj_state_ = POS_CMD_TRAJ;
            ret = runPositionCommandReference(u);
          }
          else if (param_.use_ref_txt && command_mode_enabled_ && flightReady())
          {
            ret = runTxtReference(u);
          }
          else
          {
            ret = runHover(u);
          }
        }
        else
        {
          ret = runTakeoff(now, u);
        }
        break;
      }
      case POLY_TRAJ:
      {
        if (command_mode_enabled_ && polyTrajValid(now))
        {
          ret = runPolyTrajectory(now, u);
        }
        else
        {
          exec_traj_state_ = HOVER;
          setHoverWithOdom();
          ret = runHover(u);
        }
        break;
      }
      case POS_CMD_TRAJ:
      {
        if (command_mode_enabled_ && polyTrajValid(now))
        {
          exec_traj_state_ = POLY_TRAJ;
          ret = runPolyTrajectory(now, u);
        }
        else if (command_mode_enabled_ && posCmdValid(now))
        {
          ret = runPositionCommandReference(u);
        }
        else
        {
          exec_traj_state_ = HOVER;
          setHoverWithOdom();
          ret = runHover(u);
        }
        break;
      }
      case TAKEOFF:
      {
        ret = runTakeoff(now, u);
        break;
      }
      case LAND:
      {
        ret = runLand(now, u);
        break;
      }
      default:
      {
        exec_traj_state_ = HOVER;
        setHoverWithOdom();
        ret = runHover(u);
        break;
      }
    }

    if (!ret)
    {
      ROS_ERROR_THROTTLE(0.5, "[MPCctrl] MPC solve failed, falling back to hover.");
      exec_traj_state_ = HOVER;
      setHoverWithOdom();
      ret = runHover(u);
      if (!ret)
      {
        u = idleOutput();
      }
    }

    sendCommand(u);
    publishTrackingPerformance(u);
    publishPx4State();

    if (enable_thrust_estimation_ && flightReady() && exec_traj_state_ != TAKEOFF && exec_traj_state_ != LAND)
    {
      ommpc_controller_.estimateThrustModel(imu_data_.a);
    }

    if (ros::ok())
    {
      exec_timer_.start();
    }
  }

public:
  void init(ros::NodeHandle &nh)
  {
    exec_traj_state_ = HOVER;

    readParam(nh, "control_frequency", ctrl_freq_);
    readParam(nh, "enu_frame", enu_frame_);
    readParam(nh, "vel_in_body", vel_in_body_);
    readParam(nh, "auto_start_command", auto_start_command_);
    readParam(nh, "use_pos_cmd_fallback", use_pos_cmd_fallback_);
    readParam(nh, "enable_thrust_estimation", enable_thrust_estimation_);
    readParam(nh, "min_thrust_signal", min_thrust_signal_);
    readParam(nh, "max_thrust_signal", max_thrust_signal_);
    readParam(nh, "msg_timeout/odom", odom_timeout_);
    readParam(nh, "msg_timeout/imu", imu_timeout_);
    readParam(nh, "msg_timeout/pos_cmd", pos_cmd_timeout_);

    readParam(nh, "auto_takeoff_land/enable", auto_takeoff_land_enable_);
    readParam(nh, "auto_takeoff_land/enable_auto_arm", auto_arm_enable_);
    readParam(nh, "auto_takeoff_land/takeoff_height", param_.takeoff_height);
    readParam(nh, "auto_takeoff_land/takeoff_land_speed", param_.takeoff_land_speed);
    readParam(nh, "auto_takeoff_land/motors_spinup_time", takeoff_spinup_time_);
    readParam(nh, "auto_takeoff_land/trigger_delay", takeoff_trigger_delay_);
    readParam(nh, "auto_takeoff_land/reach_tolerance", takeoff_reach_tol_);
    readParam(nh, "auto_takeoff_land/max_takeoff_velocity", max_takeoff_velocity_);
    readParam(nh, "auto_takeoff_land/auto_trigger_after_takeoff", auto_trigger_after_takeoff_);
    readParam(nh, "auto_takeoff_land/land_descent_height", land_descent_height_);
    readParam(nh, "auto_takeoff_land/land_detect_velocity", land_detect_velocity_);
    readParam(nh, "auto_takeoff_land/land_detect_keep_time", land_detect_keep_time_);

    readParam(nh, "ref_txt/enable", param_.use_ref_txt);
    readParam(nh, "ref_txt/time_step", param_.ref_time_step);
    readParam(nh, "ref_txt/ref_filename", param_.ref_filename);
    readParam(nh, "hover_percentage", param_.hover_percent);
    readParam(nh, "use_fix_yaw", param_.use_fix_yaw);
    readParam(nh, "use_trajectory_ending_pos", param_.use_trajectory_ending_pos);
    readParam(nh, "MPC_params/Q_pos_xy", param_.Q_pos_xy);
    readParam(nh, "MPC_params/Q_pos_z", param_.Q_pos_z);
    readParam(nh, "MPC_params/Q_attitude_rp", param_.Q_attitude_rp);
    readParam(nh, "MPC_params/Q_attitude_yaw", param_.Q_attitude_yaw);
    readParam(nh, "MPC_params/Q_velocity", param_.Q_velocity);
    readParam(nh, "MPC_params/R_thrust", param_.R_thrust);
    readParam(nh, "MPC_params/R_pitchroll", param_.R_pitchroll);
    readParam(nh, "MPC_params/R_yaw", param_.R_yaw);
    readParam(nh, "MPC_params/min_thrust", param_.min_thrust);
    readParam(nh, "MPC_params/max_thrust", param_.max_thrust);
    readParam(nh, "MPC_params/max_bodyrate_xy", param_.max_bodyrate_xy);
    readParam(nh, "MPC_params/max_bodyrate_z", param_.max_bodyrate_z);
    readParam(nh, "MPC_params/state_cost_exponential", param_.state_cost_exponential);
    readParam(nh, "MPC_params/input_cost_exponential", param_.input_cost_exponential);
    readParam(nh, "MPC_params/step_T", param_.step_T);

    command_mode_enabled_ = auto_start_command_;

    cmd_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    traj_start_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    px4_state_pub_ = nh.advertise<quadrotor_msgs::Px4State>("/px4_state", 10);
    tracking_perf_pub_ = nh.advertise<quadrotor_msgs::TrakingPerformance>("tracking_performance", 20);

    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_srv_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 100, &OMMPC_EXAMPLE::odomCallback, this,
                                                 ros::TransportHints().tcpNoDelay());
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("imu", 100, &OMMPC_EXAMPLE::imuCallback, this,
                                              ros::TransportHints().tcpNoDelay());
    state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &OMMPC_EXAMPLE::stateCallback, this);
    extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10,
                                                                   &OMMPC_EXAMPLE::extendedStateCallback, this);
    mpc_traj_sub_ = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>("traj", 100,
                                                                       &OMMPC_EXAMPLE::trajectoryCallback, this,
                                                                       ros::TransportHints().tcpNoDelay());
    pos_cmd_sub_ = nh.subscribe<quadrotor_msgs::PositionCommand>("pos_cmd", 100,
                                                                 &OMMPC_EXAMPLE::positionCommandCallback, this,
                                                                 ros::TransportHints().tcpNoDelay());
    takeoff_land_sub_ = nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land", 10,
                                                                  &OMMPC_EXAMPLE::takeoffLandCallback, this,
                                                                  ros::TransportHints().tcpNoDelay());

    int trials = 0;
    while (ros::ok() && !state_.connected)
    {
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      if (trials++ > 5)
      {
        ROS_ERROR_THROTTLE(2.0, "[MPCctrl] Waiting for PX4 connection.");
      }
    }

    state_change_cb_type_ = boost::bind(&OMMPC_EXAMPLE::stateChangeCallback, this, _1, _2);
    state_change_server_.setCallback(state_change_cb_type_);
    command_mode_enabled_ = auto_start_command_;

    if (param_.use_ref_txt && !readDataFromFile())
    {
      ROS_ERROR("[MPCctrl] ref_txt enabled but trajectory file cannot be loaded.");
      param_.use_ref_txt = false;
    }

    quad_positions_.resize(nstep + 1);
    quad_velocities_.resize(nstep + 1);
    yaws_.resize(nstep + 1);

    ommpc_controller_.init(param_);
    ROS_INFO("[MPCctrl] Initialized. odom=~odom traj=~traj pos_cmd=~pos_cmd takeoff_land=~takeoff_land command_mode=%d vel_in_body=%d.",
             static_cast<int>(command_mode_enabled_),
             static_cast<int>(vel_in_body_));

    exec_timer_ = nh.createTimer(ros::Duration(1.0 / std::max(1.0, ctrl_freq_)),
                                 &OMMPC_EXAMPLE::execFSMCallback,
                                 this,
                                 false,
                                 true);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ommpc_controller_node");
  ros::NodeHandle nh("~");

  OMMPC_EXAMPLE ommpc_example;
  ommpc_example.init(nh);

  ros::spin();
  return 0;
}
