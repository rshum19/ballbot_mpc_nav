
#include "ballbot_mpc_nav/ballbot_mpc.h"
#include "linear_mpc/linear_mpc.h"

// System Dynamics
#include "ballbot_mpc_nav/DroneState.h"
#include "ballbot_mpc_nav/ballbot_dynamics.h"
#include "ballbot_mpc_nav/drone_dynamics.h"
#include "ballbot_mpc_nav/tumbller_dynamics.h"

#include <ballbot_math/functions.h>
#include <ballbot_math/vector_algebra.h>

#include <iostream>
#include <math.h>
#include <vector>

namespace ballbot {
namespace control {

//========================================================================================
BallbotMPC::BallbotMPC(ros::NodeHandle &nh) : m_nh(nh) {

  // Advertise ROS Topics
  m_cmd_pub = m_nh.advertise<rt_msgs::OlcCmd>("/rt/olc_cmd", 20);
  m_ref_traj_pub =
      m_nh.advertise<ballbot_mpc_nav::DroneState>("/mpc/debug/ref_traj", 20);
  m_opt_traj_pub =
      m_nh.advertise<ballbot_mpc_nav::DroneState>("/mpc/debug/opt_traj", 20);

  // Subscribe ROS Topics
  m_odom_sub = m_nh.subscribe("/rt/odom", 2, &BallbotMPC::odom_callback, this);

  /// Load system dynamics
  ::dynamics::DroneDynamics dynamics;
  ::dynamics::BallbotDynamics ballbot_dyn;

  std::cout << "Ad:\n" << ballbot_dyn.Ad << std::endl;
  std::cout << "Bd: \n" << ballbot_dyn.Bd << std::endl;

  // Setup MPC solver
  m_Nx = dynamics.Ad.rows();
  m_Nu = dynamics.Bd.cols();
  m_N = 10;
  m_dt = 0.01;

  m_q_curr.resize(m_Nx, 1);
  Eigen::MatrixXd ref_traj = generate_reference_trajectory(0);
  m_q_curr = ref_traj.col(0);

  m_mpc.reset(new ::control::mpc::LinearMPC(
      dynamics.Ad, dynamics.Bd, dynamics.Qx, dynamics.Qn, dynamics.Ru,
      dynamics.xbounds, dynamics.ubounds, m_N));
}

//========================================================================================
void BallbotMPC::odom_callback(const rt_msgs::Odom::ConstPtr &msg) {

  m_odom_msg = *msg;

  // Need to convert to Eigen::Matrix

  m_q_curr.setZero();
  /*m_q_curr(0) = msg->xPos;
  m_q_curr(1) = msg->yPos;
  m_q_curr(2) = msg->xAng;
  m_q_curr(3) = msg->yAng;
  m_q_curr(4) = msg->xVel;
  m_q_curr(5) = msg->yVel;
  m_q_curr(6) = msg->xAngVel;
  m_q_curr(7) = msg->yAngVel;*/
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

  Eigen::MatrixXd ref_traj = generate_reference_trajectory(time);

  // Eigen::VectorXd x0 = ref_traj.col(0);

  // Solve the QP
  Eigen::MatrixXd x_out;
  double f_val;
  m_mpc->solve(m_q_curr, ref_traj, x_out, f_val);

  Eigen::MatrixXd opt_traj;
  Eigen::MatrixXd u0;
  m_mpc->get_output(x_out, u0, opt_traj);

  ballbot_mpc_nav::DroneState ref_traj_msg;
  ref_traj_msg.xPosition = ballbot::math::eig2vec(ref_traj.row(0));
  ref_traj_msg.yPosition = ballbot::math::eig2vec(ref_traj.row(1));
  ref_traj_msg.zPosition = ballbot::math::eig2vec(ref_traj.row(2));
  ref_traj_msg.xVelocity = ballbot::math::eig2vec(ref_traj.row(3));
  ref_traj_msg.yVelocity = ballbot::math::eig2vec(ref_traj.row(4));
  ref_traj_msg.zVelocity = ballbot::math::eig2vec(ref_traj.row(5));
  m_ref_traj_pub.publish(ref_traj_msg);

  // publish_state_msg(ref_traj, m_ref_traj_pub);
  publish_state_msg(opt_traj, m_opt_traj_pub);

  // Publish commands
  m_cmd_pub.publish(m_olc_cmd_msg);

  // Update to current positon (for now its fake)
  m_q_curr = ref_traj.col(1);
}

//========================================================================================
void BallbotMPC::publish_state_msg(const Eigen::MatrixXd &state,
                                   ros::Publisher &pub) {

  ballbot_mpc_nav::DroneState ref_traj_msg;
  ref_traj_msg.header.stamp = ros::Time::now();
  ref_traj_msg.xPosition = ballbot::math::eig2vec(state.row(0));
  ref_traj_msg.yPosition = ballbot::math::eig2vec(state.row(1));
  ref_traj_msg.zPosition = ballbot::math::eig2vec(state.row(2));
  ref_traj_msg.xVelocity = ballbot::math::eig2vec(state.row(3));
  ref_traj_msg.yVelocity = ballbot::math::eig2vec(state.row(4));
  ref_traj_msg.zVelocity = ballbot::math::eig2vec(state.row(5));
  pub.publish(ref_traj_msg);
}

} // namespace control
} // namespace ballbot