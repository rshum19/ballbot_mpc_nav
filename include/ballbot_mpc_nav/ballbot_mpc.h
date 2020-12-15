#pragma once

#include "ballbot_mpc_nav/ballbot_mpc.h"
#include "linear_mpc/linear_mpc.h"

#include "ballbot_mpc_nav/ballbot_dynamics.h"
#include "ballbot_mpc_nav/drone_dynamics.h"
#include "ballbot_mpc_nav/tumbller_dynamics.h"

// Ballbot ROS Msgs
#include "rt_msgs/Odom.h"
#include "rt_msgs/OlcCmd.h"

// ROS
#include <ros/ros.h>

#include <Eigen/Dense>

namespace ballbot {
namespace control {

/**
 * @brief
 */
class BallbotMPC {

public:
  /**
   * @brief Class constructor
   */
  BallbotMPC(ros::NodeHandle &nh, const double &N_horizon = 10,
             const double &mpc_dt = 0.01);
  ~BallbotMPC() = default;

  /**
   * @brief Generate ballbot dynamics
   */
  void generate_linear_dynamics();

  /**
   * @brief
   */
  void step(const double &time);

  /**
   * @brief Generate the reference trajectory to track
   */
  Eigen::MatrixXd generate_reference_trajectory(const double &time);

private:
  // Callback functions
  void odom_callback(const rt_msgs::Odom::ConstPtr &msg);

  /**
   * @brief Publish to topic
   */
  void publish_state_msg(const Eigen::MatrixXd &state, ros::Publisher &pub);

  /**
   * @brief Interpert output of QP solver and publish commands to ballbot
   */
  void publish_command(const Eigen::MatrixXd &opt_traj,
                       const Eigen::MatrixXd &u0);

  /**
   * @brief Function to publish a desired odometry, this is used for debugging
   * purposes
   */
  void publish_des_odom(const Eigen::VectorXd &des_odom);

  // ROS Node
  ros::NodeHandle m_nh;

  // ROS Topics & Msgs
  ros::Publisher m_cmd_pub;
  ros::Subscriber m_odom_sub;

  // Debug Topics
  ros::Publisher m_ref_traj_pub;
  ros::Publisher m_opt_traj_pub;
  ros::Publisher m_des_odom_pub;
  ros::Publisher m_debug_cmd_pub;

  rt_msgs::Odom m_odom_msg, m_odom_des_msg;
  rt_msgs::OlcCmd m_olc_cmd_msg;

  // MPC variables
  Eigen::VectorXd m_q_curr;
  Eigen::MatrixXd m_ref_traj;

  int m_N;
  int m_Nx;
  int m_Nu;
  int m_Nq;
  double m_dt;

  // Linear MPC instance
  std::shared_ptr<::control::mpc::LinearMPC> m_mpc;

  // Dynamic model
  //::dynamics::DroneDynamics m_dynamics;
  ::dynamics::BallbotDynamics m_dynamics;
  ::dynamics::BallbotDynamics::Trajectories m_trajectory;
};

} // namespace control
} // namespace ballbot
