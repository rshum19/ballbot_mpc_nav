#include "ballbot_mpc_nav/ballbot_mpc.h"

#include <ros/ros.h>

// C++
#include <chrono>
#include <iostream>

int main(int argc, char **argv) {

  // Set up ROS
  ros::init(argc, argv, "ballbot_mpc_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;

  double control_rate_hz = 100;

  ros::Rate loop_rate(control_rate_hz);

  // Initialize mpc;
  ballbot::control::BallbotMPC ballbot_mpc(nh);
  int i = 0;

  ros::Time loop_start = ros::Time::now();
  while (ros::ok()) {

    // Get robot states

    // Get control command
    ros::Time t_start = ros::Time::now();
    double t_now = ros::Time::now().toSec() - loop_start.toSec();
    ballbot_mpc.step(t_now);
    ros::Time t_elapsed = ros::Time::now();

    double duration = t_elapsed.toSec() - t_start.toSec();

    ROS_INFO_STREAM("iteration: " << i << ": " << duration);
    i++;

    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}