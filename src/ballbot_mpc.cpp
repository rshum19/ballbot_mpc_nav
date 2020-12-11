
#include "ballbot_mpc_nav/ballbot_mpc.h"
#include "linear_mpc/linear_mpc.h"

// System Dynamics
#include "ballbot_mpc_nav/drone_dynamics.h"
#include "ballbot_mpc_nav/tumbller_dynamics.h"

#include <iostream>
#include <math.h>

namespace ballbot {
namespace control {

//========================================================================================
BallbotMPC::BallbotMPC(ros::NodeHandle &nh) : m_nh(nh) {

  // Advertise ROS Topics
  m_cmd_pub = m_nh.advertise<rt_msgs::OlcCmd>("/rt/olc_cmd", 20);

  // Subscribe ROS Topics
  m_odom_sub = m_nh.subscribe("/rt/odom", 2, &BallbotMPC::odom_callback, this);

  // Setup MPC solver
  m_Nx = ::control::mpc::m_Nx;
  m_Nu = ::control::mpc::m_Nu;
  m_N = ::control::mpc::m_N;
  m_dt = 0.01;

  /// Load system dynamics
  ::dynamics::DroneDynamics dynamics;

  m_q_curr.resize(m_Nx, 1);

  m_mpc.reset(new ::control::mpc::LinearMPC(
      dynamics.Ad, dynamics.Bd, dynamics.Qx, dynamics.Qn, dynamics.Ru,
      dynamics.xbounds, dynamics.ubounds));
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

  Eigen::VectorXd x0 = ref_traj.col(0);

  // Solve the QP
  Eigen::MatrixXd x_out;
  double f_val;
  m_mpc->solve(x0, ref_traj, x_out, f_val);

  // Publish commands
  m_cmd_pub.publish(m_olc_cmd_msg);
}

} // namespace control
} // namespace ballbot