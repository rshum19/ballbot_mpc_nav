#pragma once

#include <Eigen/Dense>
#include <math.h>

#include <iostream>

namespace dynamics {

class BallbotDynamics {

public:
  /**
   * @brief Constructor
   */
  BallbotDynamics() {

    // A matrix
    Ad.resize(Nx, Nx);
    Ad.row(0) << 0, 0, 1, 0;
    Ad.row(1) << 0, 0, 0, 1;
    Ad.row(2) << 0, Gamma1, 0, 0;
    Ad.row(3) << 0, Gamma2, 0, 0;

    // B matrix
    Bd.resize(Nx, Nu);
    Bd.col(0) << 0, 0, Beta1, Beta2;

    // Weights on state deviation and control input
    Qx.resize(Nx, Nx);
    Qx.setZero();
    Qx.diagonal() << 100, 100, 1, 100;

    Qn.resize(Nx, Nx);
    Qn = 10 * Qx;

    Ru.resize(Nu, Nu);
    Ru.setZero();
    Ru.diagonal() << 1;

    // Bounds on states and controls
    xbounds.resize(Nx, 2);
    xbounds.col(0) << -1e10, -M_PI / 4, -1e10, -1e10;
    xbounds.col(1) << 1e10, M_PI / 4, 1e10, 1e10;

    ubounds.resize(Nu, 2);
    ubounds.col(0) << -10;
    ubounds.col(1) << 10;
  }

  /**
   * @brief Generate the reference trajectory to track
   *
   * @param[in] tstart  time to start motion from [sec]
   * @param[in] N motion horizon length [qty]
   * @param[in] dt  time discretization [sec]
   *
   * @return the refernce trajectory matrix of size [Nx,N] where the rows
   * represent the states [theta,phi,dtheta,dphi]'
   */
  Eigen::MatrixXd generate_reference_trajectory(const double &time,
                                                const int &N,
                                                const double &dt) {

    // Reference trajectory
    Eigen::MatrixXd ref_traj(Nx, N + 1);
    ref_traj.setZero();
    double xPos = 0.0;
    for (int i = 0; i < N + 1; i++) {
      double ti = time + dt * i;
      // ref_traj(0, i) = sin(0.5 * ti); // xref
      xPos = sin(0.5 * ti);
      ref_traj(0, i) = xPos / r;
      ref_traj(1, i) = 0; // yref
    }

    return ref_traj;
  }

  // Linearized dynamics
  double g = 9.81;          // acceleration due to gravity [kg m/s^2]
  double mball = 2.437;     // Mass of ball [kg]
  double mbody = 51.663126; // Mass of body [kg]
  double Ibody = 12.5905;   // Inertia of body [kg m^2]
  double Iball = 0.0174;    // Inertia of ball [kg m^2]
  double l = 0.69;          // Distance from center of ball to body com [m]
  double r = 0.10583037;    // Radius of ball [m]

  int Nx = 4;
  int Nu = 1;

  double denom = Iball * Ibody + Iball * std::pow(l, 2) * mbody +
                 Ibody * (mbody + mball) * std::pow(r, 2) +
                 std::pow(l, 2) * mball * mbody * std::pow(r, 2);

  double alpha = Iball + (mball + mbody) * std::pow(r, 2);
  double beta = mbody * r * l;
  double gamma = Ibody + mbody * std::pow(l, 2);
  /*double denom =
      Ibody * alpha + Iball * mbody * std::pow(l, 2) + beta * mball * r * l;*/

  double Gamma1 = -(g * l * mbody * (alpha + beta)) / denom;
  double Gamma2 = (g * l * mbody * alpha) / denom;
  double Beta1 = (alpha + gamma + 2 * beta) / denom;
  double Beta2 = -(alpha + beta) / denom;

  // Linear Dynamics
  Eigen::MatrixXd Ad;
  Eigen::MatrixXd Bd;

  // State and Control bounds
  Eigen::MatrixXd xbounds;
  Eigen::MatrixXd ubounds;

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx;
  Eigen::MatrixXd Qn;
  Eigen::MatrixXd Ru;
};

} // namespace dynamics