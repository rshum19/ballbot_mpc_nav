#include "ballbot_mpc_nav/ballbot_mpc.h"

#include <ros/ros.h>

// C++
#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char **argv) {

  // Set up ROS
  ros::init(argc, argv, "ballbot_mpc_node2");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;

  // Read params
  std::string node_name = ros::this_node::getName();
  double N_horizon;
  if (!nh.getParam(node_name + "/horizon_size", N_horizon)) {
    ROS_ERROR_STREAM("[BallbotMPCNode] could not find parameter "
                     "/horizon_size in namespace!");
  }
  ROS_INFO_STREAM(
      "[BallbotMPCNode] loaded paremeter /N_horizon= " << N_horizon);

  double control_freq_hz;
  if (!nh.getParam(node_name + "/control_freq_hz", control_freq_hz)) {
    ROS_ERROR_STREAM("[BallbotMPCNode] could not find parameter "
                     "/control_freq_hz in namespace!");
  }
  ROS_INFO_STREAM("[BallbotMPCNode] loaded paremeter /control_freq_hz = "
                  << control_freq_hz);

  double mpc_dt;
  if (!nh.getParam(node_name + "/mpc_dt", mpc_dt)) {
    ROS_ERROR_STREAM("[BallbotMPCNode] could not find parameter "
                     "/mpc_dt in namespace!");
  }
  ROS_INFO_STREAM("[BallbotMPCNode] loaded paremeter /mpc_dt = " << mpc_dt);

  ros::Rate loop_rate(control_freq_hz);

  // Initialize mpc;
  ballbot::control::BallbotMPC ballbot_mpc(nh);

  // Initialize main control loop
  int i = 0;
  ros::Time loop_start = ros::Time::now();
  while (ros::ok()) {

    // Get control command
    ros::Time t_start = ros::Time::now();
    double t_now = ros::Time::now().toSec() - loop_start.toSec();
    ballbot_mpc.step(t_now);
    ROS_INFO_STREAM("Tnow: " << t_now);
    ros::Time t_elapsed = ros::Time::now();

    double duration = t_elapsed.toSec() - t_start.toSec();

    ROS_DEBUG_STREAM("iteration: " << i << ": " << duration);
    i++;

    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}
