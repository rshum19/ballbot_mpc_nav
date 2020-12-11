#pragma once

#include <Eigen/Dense>
#include <cmath>

#include <iostream>

namespace dynamics {

const double INF = 1000000;

class TumbllerDynamics {

public:
  /**
   * @brief Constructor
   */
  TumbllerDynamics() {

    // A matrix
    Ad.resize(Nx, Nx);
    Ad.row(0) << 0, 1, 0, 0;
    Ad.row(1) << 0, -(Ipend + mpend * std::pow(l, 2)) * f / denom,
        std::pow(mpend, 2) * g * std::pow(l, 2) / denom, 0;
    Ad.row(2) << 0, 0, 0, 1;
    Ad.row(3) << 0, -mpend * l * f / denom,
        mpend * g * l * (mcart * mpend) / denom, 0;

    // B matrix
    Bd.resize(Nx, Nu);
    Bd.col(0) << 0, Ipend + mpend * std::pow(l, 2) / denom, 0,
        mpend * l / denom;

    // Weights on state deviation and control input
    Qx.resize(Nx, Nx);
    Qx.setZero();
    Qx.diagonal() << 100, 100, 10, 10;

    Qn.resize(Nx, Nx);
    Qn = 10 * Qx;

    Ru.resize(Nu, Nu);
    Ru.setZero();
    Ru.diagonal() << 1;

    // Bounds on states and controls
    xbounds.resize(Nx, 2);
    xbounds.col(0) << -INF, -INF, -M_PI / 4, INF;
    xbounds.col(1) << INF, INF, M_PI / 4, INF;

    ubounds.resize(Nu, 2);
    ubounds.col(0) << -10;
    ubounds.col(1) << 10;
  }

  // Linearized dynamics
  double g = 9.81;
  double mcart = 0.493;
  double mpend = 0.312;
  double Ipend = 0.00024;
  double l = 0.04;
  double f = 0.01;

  int Nx = 4;
  int Nu = 1;

  double denom = Ipend * (mcart * mpend) + mcart * mpend * std::pow(l, 2);

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