#pragma once

#include <Eigen/Dense>
#include <cmath>

#include <iostream>

namespace dynamics {

class DroneDynamics {

public:
  /**
   * @brief Constructor
   */
  DroneDynamics() {

    // A matrix
    Ad.resize(Nx, Nx);
    Ad.row(0) << 1, 0, 0, dt, 0, 0;
    Ad.row(1) << 0, 1, 0, 0, dt, 0;
    Ad.row(2) << 0, 0, 1, 0, 0, dt;
    Ad.row(3) << 0, 0, 0, 1, 0, 0;
    Ad.row(4) << 0, 0, 0, 0, 1, 0;
    Ad.row(5) << 0, 0, 0, 0, 0, 1;

    // B matrix
    Bd.resize(Nx, Nu);
    Bd.row(0) << 0, 0, 0, 0, 0;
    Bd.row(1) << 0, 0, 0, 0, 0;
    Bd.row(2) << 0, 0, 0, 0, 0;
    Bd.row(3) << -g * dt, 0, 0, 0, 0;
    Bd.row(4) << 0, g * dt, 0, 0, 0;
    Bd.row(5) << 0, 0, 0, dt / m, -g * dt;

    // Weights on state deviation and control input
    Qx.resize(Nx, Nx);
    Qx.setZero();
    Qx.diagonal() << 100, 100, 1000, 1, 1, 1;

    Qn.resize(Nx, Nx);
    Qn = 5 * Qx;

    Ru.resize(Nu, Nu);
    Ru.setZero();
    Ru.diagonal() << 1, 1, 1, 1, 1;

    // Bounds on states and controls
    xbounds.resize(Nx, 2);
    xbounds.col(0) << -INF, -INF, -INF, -INF, -INF, -INF;
    xbounds.col(1) << INF, INF, INF, INF, INF, INF;

    ubounds.resize(Nu, 2);
    ubounds.col(0) << -M_PI / 2, -M_PI / 2, -M_PI / 2, -20, 1;
    ubounds.col(1) << M_PI / 2, M_PI / 2, M_PI / 2, 20, 1;
  }

  // Linearized dynamics
  double g = 9.81;
  double f = 0.01;
  double m = 1;
  double dt = 0.1;

  int Nx = 6;
  int Nu = 5;

  int INF = 1000000;

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