#pragma once

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <nmpc_motion_planner/casadi_mpc_template.hpp>

#include <chrono>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

namespace casadi_mpc_template
{

class MotionPlanningProb : public Problem
{
  public:
    MotionPlanningProb(DynamicsType dynamics_type, int state_dim, int control_dim, int horizon_length, double dt);
    virtual ~MotionPlanningProb() = default;

    virtual casadi::MX dynamics(casadi::MX x, casadi::MX u) override;
    virtual casadi::MX stage_cost(casadi::MX x, casadi::MX u) override;
    // virtual casadi::MX terminal_cost(casadi::MX x) override;
    Eigen::VectorXd discretized_dynamics(double dt, Eigen::VectorXd x, Eigen::VectorXd u);
    casadi::MX forward_kinematics(casadi::MX q);
    casadi::MX compute_trans_error(casadi::MX x_pose);
    casadi::MX compute_ori_error(casadi::MX x_quat);
    casadi::DM Q_trans, Q_ori, Q_vel, R;

    casadi::DM x_pose_ref, x_quat_ref;
    casadi::MX x_pose, x_quat;
};

} // namespace casadi_mpc_template
