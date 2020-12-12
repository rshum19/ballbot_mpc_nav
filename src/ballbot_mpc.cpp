
#include "ballbot_mpc_nav/ballbot_mpc.h"
#include "linear_mpc/linear_mpc.h"

// System Dynamics
#include "ballbot_mpc_nav/DroneState.h"

#include <ballbot_math/functions.h>
#include <ballbot_math/vector_algebra.h>

#include <iostream>
#include <math.h>
#include <vector>

namespace ballbot {
namespace control {

//========================================================================================
BallbotMPC::BallbotMPC(ros::NodeHandle &nh, const double &N_horizon,
                       const double &mpc_dt)
    : m_nh(nh), m_N(N_horizon), m_dt(mpc_dt) {

  // Advertise ROS Topics
  m_cmd_pub = m_nh.advertise<rt_msgs::OlcCmd>("/rt/olc_cmd", 20);
  m_ref_traj_pub =
      m_nh.advertise<rt_msgs::Odom>("/mpc/debug/ref_traj", 20);
  m_opt_traj_pub =
      m_nh.advertise<rt_msgs::Odom>("/mpc/debug/opt_traj", 20);

  m_des_odom_pub = m_nh.advertise<rt_msgs::Odom>("/mpc/debug/des_odom", 20);
  m_debug_cmd_pub = m_nh.advertise<rt_msgs::OlcCmd>("/mpc/debug/cmd",20);

  // Subscribe ROS Topics
  m_odom_sub = m_nh.subscribe("/rt/odom", 2, &BallbotMPC::odom_callback, this);

  // Setup MPC solver
  m_Nx = m_dynamics.Ad.rows();
  m_Nu = m_dynamics.Bd.cols();
  // m_N = 10;
  // m_dt = 0.01;

  m_q_curr.resize(m_Nx, 1);
  Eigen::MatrixXd ref_traj =
      m_dynamics.generate_reference_trajectory(0, m_Nx, m_dt);
  m_q_curr = ref_traj.col(0);

  m_mpc.reset(new ::control::mpc::LinearMPC(
      m_dynamics.Ad, m_dynamics.Bd, m_dynamics.Qx, m_dynamics.Qn, m_dynamics.Ru,
      m_dynamics.xbounds, m_dynamics.ubounds, m_N));
}

//========================================================================================
void BallbotMPC::odom_callback(const rt_msgs::Odom::ConstPtr &msg) {

  m_odom_msg = *msg;

  // Need to convert to Eigen::Matrix

  // m_q_curr.setZero();
  // Convert linear position and velocity to angular
  m_q_curr(0) = msg->xPos / m_dynamics.r;
  m_q_curr(1) = msg->xAng;
  m_q_curr(2) = msg->xVel / m_dynamics.r;
  m_q_curr(3) = msg->xAngVel;
}

//========================================================================================
Eigen::MatrixXd BallbotMPC::generate_reference_trajectory(const double &time) {

  // Reference trajectory
  Eigen::MatrixXd ref_traj(m_Nx, m_N + 1);
  ref_traj.setZero();

  for (int i = 0; i < m_N + 1; i++) {
    double ti = time + m_dt * i;
    ref_traj(0, i) = cos(0.5 * ti); // xref
    ref_traj(1, i) = sin(0.5 * ti); // yref
    ref_traj(2, i) = 0.1 * ti;      // zref
  }

  return ref_traj;
}
//========================================================================================
void BallbotMPC::step(const double &time) {

  Eigen::MatrixXd ref_traj =
      m_dynamics.generate_reference_trajectory(time, m_N, m_dt);

  // Eigen::VectorXd x0 = ref_traj.col(0);

  // Solve the QP
  Eigen::MatrixXd x_out;
  double f_val;
  m_mpc->solve(m_q_curr, ref_traj, x_out, f_val);

  Eigen::MatrixXd opt_traj;
  Eigen::MatrixXd u0;
  m_mpc->get_output(x_out, u0, opt_traj);

  // std::cout << "theta_opt: " << opt_traj.row(0) << std::endl;
  // std::cout << "phi_opt: " << opt_traj.row(1) << std::endl;

  publish_state_msg(ref_traj, m_ref_traj_pub);
  publish_state_msg(opt_traj, m_opt_traj_pub);

  // Publish commands
  publish_command(opt_traj, u0);

  // Update to current positon (for now its fake)
  //m_q_curr = ref_traj.col(1);
  publish_des_odom(ref_traj.col(1));
}

//========================================================================================
void BallbotMPC::publish_state_msg(const Eigen::MatrixXd &state,
                                   ros::Publisher &pub) {

  rt_msgs::Odom ref_traj_msg;
  /*ref_traj_msg.xPosition = ballbot::math::eig2vec(state.row(0));
  ref_traj_msg.yPosition = ballbot::math::eig2vec(state.row(1));
  // ref_traj_msg.zPosition = ballbot::math::eig2vec(state.row(2));
  ref_traj_msg.xVelocity = ballbot::math::eig2vec(state.row(2));
  ref_traj_msg.yVelocity = ballbot::math::eig2vec(state.row(3));
  // ref_traj_msg.zVelocity = ballbot::math::eig2vec(state.row(5));*/

  ref_traj_msg.xPos = state(0,1) * m_dynamics.r;
  ref_traj_msg.yPos = 0.0;
  ref_traj_msg.yaw = 0.0;
  ref_traj_msg.xVel = state(2,1) * m_dynamics.r;
  ref_traj_msg.yVel = 0.0;
  ref_traj_msg.xAng = state(1,1);
  ref_traj_msg.yAng = 0.0;
  ref_traj_msg.xAngVel = state(3,1);
  ref_traj_msg.yAngVel = 0.0;

  pub.publish(ref_traj_msg);
}

//========================================================================================
void BallbotMPC::publish_command(const Eigen::MatrixXd &opt_traj,
                                 const Eigen::MatrixXd &u0) {

  // Convert ball angular velocity to linear velocity
  Eigen::VectorXd x_vel = opt_traj.row(2) * m_dynamics.r;
  Eigen::VectorXd x_pos = opt_traj.row(0) * m_dynamics.r;

  rt_msgs::OlcCmd olc_cmd_msg;
  olc_cmd_msg.xAng = opt_traj(1, 1);
  olc_cmd_msg.yAng = 0.0;
  olc_cmd_msg.xVel = x_vel(1);
  olc_cmd_msg.yVel = 0.0;
	
  //m_cmd_pub.publish(olc_cmd_msg);
  m_debug_cmd_pub.publish(olc_cmd_msg);
}

//========================================================================================
void BallbotMPC::publish_des_odom(const Eigen::VectorXd &des_odom) {

  rt_msgs::Odom odom_msg;
  odom_msg.xPos = des_odom(0);
  odom_msg.yPos = 0.0;
  odom_msg.yaw = 0.0;
  odom_msg.xVel = des_odom(1);
  odom_msg.yVel = 0.0;
  odom_msg.xAng = des_odom(2);
  odom_msg.yAng = 0.0;
  odom_msg.xAngVel = des_odom(3);
  odom_msg.yAngVel = 0.0;

  m_des_odom_pub.publish(odom_msg);
}
} // namespace control
} // namespace ballbot
