/**
 * @file visual_ekf.cpp
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief Visual-based ekf estimator implementation.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <iostream>
#include <ros/console.h>

#include "visual_ekf.hpp"

Eigen::Quaterniond operator+(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    q.vec() = q1.vec() + q2.vec();
    q.w() = q1.w() + q2.w();
    return q;
}

Eigen::Quaterniond operator-(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    q.vec() = q1.vec() - q2.vec();
    q.w() = q1.w() - q2.w();
    return q;
}

Eigen::Quaterniond operator*(const double a, const Eigen::Quaterniond b)
{
    Eigen::Quaterniond c;
    c.vec() = a * b.vec();
    c.w() = a * b.w();
    return c;
}

Eigen::Quaterniond operator*(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    q.x() = q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y();  
    q.y() = q1.w() * q2.y() - q1.x() * q2.z() + q1.y() * q2.w() + q1.z() * q2.x();
    q.z() = q1.w() * q2.z() + q1.x() * q2.y() - q1.y() * q2.x() + q1.z() * q2.w();
    q.w() = q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z();
    return q;
}

Eigen::Quaterniond operator/(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    double sum = q2.x() * q2.x() + q2.y() * q2.y() + q2.z() * q2.z() + q2.w() * q2.w();
    q.w() = (q2.w() * q1.w() + q2.x() * q1.x() + q2.y() * q1.y() + q2.z() * q1.z())/sum;
    q.x() = (q2.w() * q1.x() - q2.x() * q1.w() - q2.y() * q1.z() + q2.z() * q1.y())/sum;
    q.y() = (q2.w() * q1.y() + q2.x() * q1.z() - q2.y() * q1.w() - q2.z() * q1.x())/sum;
    q.z() = (q2.w() * q1.z() - q2.x() * q1.y() + q2.y() * q1.x() - q2.z() * q1.w())/sum;
    return q;
}

VisualEKF::VisualEKF()
{
    x_.r_W.setZero();
    x_.q_WO.setIdentity();
    x_.v_O.setZero();
    x_.omega_O.setZero();
    P_.setIdentity();
}

bool VisualEKF::initialise(const ObjectState initialState)
{
    x_ = initialState;

    P_.block<3, 3>(0, 0) *= sigma_c_r_W * sigma_c_r_W;           // 1 cm
    P_.block<4, 4>(3, 3) *= sigma_c_q_WO * sigma_c_q_WO;         // quoternion error
    P_.block<3, 3>(7, 7) *= sigma_c_v_O * sigma_c_v_O;           // 1 cm/s
    P_.block<3, 3>(10, 10) *= sigma_c_omega_O * sigma_c_omega_O; // 1 degree/sec

    std::cout << "P_ init:" << std::endl << P_.diagonal() << std::endl << std::endl;

    initialised = true;

    return true;
}

bool VisualEKF::isInitialsed()
{
    return initialised;
}

VisualEKF::~VisualEKF()
{
}

ObjectStateDerivative VisualEKF::calcStateDerivative(const ObjectState &state)
{
    ObjectStateDerivative stateDerivative;

    stateDerivative.r_W_dot = state.q_WO.toRotationMatrix() * state.v_O;

    Eigen::Quaterniond temp;
    temp.w() = 0.0;
    temp.vec() = state.omega_O;
    stateDerivative.q_WO_dot = 0.5 * (state.q_WO * temp);

    stateDerivative.v_O_dot = state.q_WO.toRotationMatrix().inverse() * Eigen::Vector3d(0.0, 0.0, -9.81) - state.omega_O.cross(state.v_O);

    stateDerivative.omega_O_dot = state.inertia_inverse * (state.omega_O.cross(state.inertia * state.omega_O));

    // debug the state derivatives
    // std::cout << std::endl
    //           << "r_W_dot: " << std::endl
    //           << stateDerivative.r_W_dot << std::endl
    //           << "q_WO_dot: " << std::endl
    //           << stateDerivative.q_WO_dot.w() << std::endl
    //           << stateDerivative.q_WO_dot.vec() << std::endl
    //           << "v_O_dot: " << std::endl
    //           << stateDerivative.v_O_dot << std::endl
    //           << "omega_O_dot: " << std::endl
    //           << state.omega_O << std::endl
    //           << stateDerivative.omega_O_dot << std::endl
    //           << std::endl;

    return stateDerivative;
}

bool VisualEKF::calcJacobian(const ObjectState &state,
                             const double &dt,
                             Eigen::Matrix<double, 13, 13> &jacobian)
{
    jacobian.setIdentity();

    Eigen::Matrix3d C_WO = state.q_WO.toRotationMatrix();

    // Hard-coded jacobian matrix obtained via symbolic deriviation in Matlab.
    jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    jacobian.block<3, 3>(0, 7) << C_WO(0, 0) - (C_WO(0, 1) * dt * state.omega_O.z()) / 2 + (C_WO(0, 2) * dt * state.omega_O.y()) / 2, C_WO(0, 1) + (C_WO(0, 0) * dt * state.omega_O.z()) / 2 - (C_WO(0, 2) * dt * state.omega_O.x()) / 2, C_WO(0, 2) - (C_WO(0, 0) * dt * state.omega_O.y()) / 2 + (C_WO(0, 1) * dt * state.omega_O.x()) / 2,
        C_WO(1, 0) - (C_WO(1, 1) * dt * state.omega_O.z()) / 2 + (C_WO(1, 2) * dt * state.omega_O.y()) / 2, C_WO(1, 1) + (C_WO(1, 0) * dt * state.omega_O.z()) / 2 - (C_WO(1, 2) * dt * state.omega_O.x()) / 2, C_WO(1, 2) - (C_WO(1, 0) * dt * state.omega_O.y()) / 2 + (C_WO(1, 1) * dt * state.omega_O.x()) / 2,
        C_WO(2, 0) - (C_WO(2, 1) * dt * state.omega_O.z()) / 2 + (C_WO(2, 2) * dt * state.omega_O.y()) / 2, C_WO(2, 1) + (C_WO(2, 0) * dt * state.omega_O.z()) / 2 - (C_WO(2, 2) * dt * state.omega_O.x()) / 2, C_WO(2, 2) - (C_WO(2, 0) * dt * state.omega_O.y()) / 2 + (C_WO(2, 1) * dt * state.omega_O.x()) / 2;

    jacobian.block<3, 3>(0, 10) << (C_WO(0, 1) * dt * state.v_O.z()) / 2 - (C_WO(0, 2) * dt * state.v_O.y()) / 2, (C_WO(0, 2) * dt * state.v_O.x()) / 2 - (C_WO(0, 0) * dt * state.v_O.z()) / 2, (C_WO(0, 0) * dt * state.v_O.y()) / 2 - (C_WO(0, 1) * dt * state.v_O.x()) / 2,
        (C_WO(1, 1) * dt * state.v_O.z()) / 2 - (C_WO(1, 2) * dt * state.v_O.y()) / 2, (C_WO(1, 2) * dt * state.v_O.x()) / 2 - (C_WO(1, 0) * dt * state.v_O.z()) / 2, (C_WO(1, 0) * dt * state.v_O.y()) / 2 - (C_WO(1, 1) * dt * state.v_O.x()) / 2,
        (C_WO(2, 1) * dt * state.v_O.z()) / 2 - (C_WO(2, 2) * dt * state.v_O.y()) / 2, (C_WO(2, 2) * dt * state.v_O.x()) / 2 - (C_WO(2, 0) * dt * state.v_O.z()) / 2, (C_WO(2, 0) * dt * state.v_O.y()) / 2 - (C_WO(2, 1) * dt * state.v_O.x()) / 2;

    jacobian.block<4, 4>(3, 3) << (dt * state.omega_O.x()) / 4 - (dt * ((dt * state.omega_O.x() * state.omega_O.y()) / 2 + (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4 + 1, -(dt * state.omega_O.y()) / 4 - (dt * ((dt * state.omega_O.z() * state.omega_O.z()) / 2 + state.omega_O.x() * ((dt * state.omega_O.x()) / 2 + 1))) / 4, -(dt * state.omega_O.z()) / 4 - (dt * (state.omega_O.y() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4, -(dt * ((dt * state.omega_O.y() * state.omega_O.y()) / 2 + state.omega_O.z() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.x() * state.omega_O.z()) / 2)) / 4,
        (dt * state.omega_O.y()) / 4 + (dt * ((dt * state.omega_O.z() * state.omega_O.z()) / 2 + state.omega_O.x() * ((dt * state.omega_O.x()) / 2 + 1))) / 4, (dt * state.omega_O.x()) / 4 - (dt * ((dt * state.omega_O.x() * state.omega_O.y()) / 2 + (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4 + 1, (dt * ((dt * state.omega_O.y() * state.omega_O.y()) / 2 + state.omega_O.z() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.x() * state.omega_O.z()) / 2)) / 4, -(dt * state.omega_O.z()) / 4 - (dt * (state.omega_O.y() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4,
        (dt * state.omega_O.z()) / 4 + (dt * (state.omega_O.y() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4, -(dt * ((dt * state.omega_O.y() * state.omega_O.y()) / 2 + state.omega_O.z() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.x() * state.omega_O.z()) / 2)) / 4, (dt * state.omega_O.x()) / 4 - (dt * ((dt * state.omega_O.x() * state.omega_O.y()) / 2 + (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4 + 1, (dt * state.omega_O.y()) / 4 + (dt * ((dt * state.omega_O.z() * state.omega_O.z()) / 2 + state.omega_O.x() * ((dt * state.omega_O.x()) / 2 + 1))) / 4,
        (dt * ((dt * state.omega_O.y() * state.omega_O.z()) / 2 + state.omega_O.z() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.x() * state.omega_O.z()) / 2)) / 4, (dt * state.omega_O.z()) / 4 + (dt * (state.omega_O.y() * ((dt * state.omega_O.x()) / 2 + 1) - (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4, -(dt * state.omega_O.y()) / 4 - (dt * ((dt * state.omega_O.z() * state.omega_O.z()) / 2 + state.omega_O.x() * ((dt * state.omega_O.x()) / 2 + 1))) / 4, (dt * state.omega_O.x()) / 4 - (dt * ((dt * state.omega_O.x() * state.omega_O.y()) / 2 + (dt * state.omega_O.y() * state.omega_O.z()) / 2)) / 4 + 1;

    jacobian.block<4, 3>(3, 10) << (state.q_WO.x() * dt) / 4 - (dt * (state.q_WO.y() + (dt * (state.q_WO.x() * state.omega_O.y() + state.q_WO.y() * state.omega_O.x() - state.q_WO.w() * state.omega_O.z())) / 2 + (state.q_WO.y() * dt * state.omega_O.x()) / 2 + (state.q_WO.z() * dt * state.omega_O.y()) / 2 + (state.q_WO.w() * dt * state.omega_O.z()) / 2)) / 4, -(state.q_WO.y() * dt) / 4 - (dt * (state.q_WO.z() + (dt * (state.q_WO.x() * state.omega_O.z() + state.q_WO.z() * state.omega_O.x() + state.q_WO.w() * state.omega_O.y())) / 2 + (state.q_WO.x() * dt * state.omega_O.x()) / 2 - (state.q_WO.z() * dt * state.omega_O.z()) / 2 + (state.q_WO.w() * dt * state.omega_O.y()) / 2)) / 4, -(state.q_WO.z() * dt) / 4 - (dt * (state.q_WO.w() + (dt * (state.q_WO.y() * state.omega_O.z() - state.q_WO.z() * state.omega_O.y() + state.q_WO.w() * state.omega_O.x())) / 2 + (state.q_WO.x() * dt * state.omega_O.y()) / 2 + (state.q_WO.y() * dt * state.omega_O.z()) / 2 - (state.q_WO.w() * dt * state.omega_O.x()) / 2)) / 4,
        (state.q_WO.y() * dt) / 4 + (dt * (state.q_WO.x() - (dt * (state.q_WO.y() * state.omega_O.y() - state.q_WO.x() * state.omega_O.x() + state.q_WO.z() * state.omega_O.z())) / 2 + (state.q_WO.x() * dt * state.omega_O.x()) / 2 + (state.q_WO.z() * dt * state.omega_O.z()) / 2 - (state.q_WO.w() * dt * state.omega_O.y()) / 2)) / 4, (state.q_WO.x() * dt) / 4 - (dt * (state.q_WO.w() + (dt * (state.q_WO.y() * state.omega_O.z() - state.q_WO.z() * state.omega_O.y() + state.q_WO.w() * state.omega_O.x())) / 2 + (state.q_WO.y() * dt * state.omega_O.x()) / 2 - (state.q_WO.z() * dt * state.omega_O.y()) / 2 - (state.q_WO.w() * dt * state.omega_O.z()) / 2)) / 4, (dt * (state.q_WO.z() + (dt * (state.q_WO.x() * state.omega_O.z() + state.q_WO.z() * state.omega_O.x() + state.q_WO.w() * state.omega_O.y())) / 2 + (state.q_WO.x() * dt * state.omega_O.z()) / 2 - (state.q_WO.y() * dt * state.omega_O.y()) / 2 - (state.q_WO.z() * dt * state.omega_O.x()) / 2)) / 4 - (state.q_WO.w() * dt) / 4,
        (state.q_WO.z() * dt) / 4 + (dt * (state.q_WO.w() + (dt * (state.q_WO.y() * state.omega_O.z() - state.q_WO.z() * state.omega_O.y() + state.q_WO.w() * state.omega_O.x())) / 2 + (state.q_WO.x() * dt * state.omega_O.y()) / 2 - (state.q_WO.y() * dt * state.omega_O.z()) / 2 + (state.q_WO.w() * dt * state.omega_O.x()) / 2)) / 4, (state.q_WO.w() * dt) / 4 - (dt * ((dt * (state.q_WO.y() * state.omega_O.y() - state.q_WO.x() * state.omega_O.x() + state.q_WO.z() * state.omega_O.z())) / 2 - state.q_WO.x() + (state.q_WO.x() * dt * state.omega_O.z()) / 2 + (state.q_WO.y() * dt * state.omega_O.y()) / 2 + (state.q_WO.z() * dt * state.omega_O.x()) / 2)) / 4, (state.q_WO.x() * dt) / 4 - (dt * (state.q_WO.y() + (dt * (state.q_WO.x() * state.omega_O.y() + state.q_WO.y() * state.omega_O.x() - state.q_WO.w() * state.omega_O.z())) / 2 - (state.q_WO.y() * dt * state.omega_O.x()) / 2 + (state.q_WO.z() * dt * state.omega_O.y()) / 2 - (state.q_WO.w() * dt * state.omega_O.z()) / 2)) / 4,
        (state.q_WO.w() * dt) / 4 - (dt * (state.q_WO.z() + (dt * (state.q_WO.x() * state.omega_O.z() + state.q_WO.z() * state.omega_O.x() + state.q_WO.w() * state.omega_O.y())) / 2 - (state.q_WO.x() * dt * state.omega_O.z()) / 2 - (state.q_WO.y() * dt * state.omega_O.y()) / 2 + (state.q_WO.z() * dt * state.omega_O.x()) / 2)) / 4, (dt * (state.q_WO.y() + (dt * (state.q_WO.x() * state.omega_O.y() + state.q_WO.y() * state.omega_O.x() - state.q_WO.w() * state.omega_O.z())) / 2 + (state.q_WO.x() * dt * state.omega_O.y()) / 2 - (state.q_WO.y() * dt * state.omega_O.z()) / 2 - (state.q_WO.w() * dt * state.omega_O.x()) / 2)) / 4 - (state.q_WO.z() * dt) / 4, (state.q_WO.y() * dt) / 4 - (dt * ((dt * (state.q_WO.y() * state.omega_O.y() - state.q_WO.x() * state.omega_O.x() + state.q_WO.z() * state.omega_O.z())) / 2 - state.q_WO.x() + (state.q_WO.x() * dt * state.omega_O.x()) / 2 + (state.q_WO.z() * dt * state.omega_O.z()) / 2 + (state.q_WO.w() * dt * state.omega_O.y()) / 2)) / 4;

    jacobian.block<3, 3>(7, 7) << 1 - (dt * (dt * state.omega_O.y() * state.omega_O.y() + dt * state.omega_O.z() * state.omega_O.z())) / 2, (dt * state.omega_O.z()) / 2 + (dt * (state.omega_O.z() + dt * state.omega_O.x() * state.omega_O.y())) / 2, -(dt * state.omega_O.y()) / 2 - (dt * (state.omega_O.y() - dt * state.omega_O.x() * state.omega_O.z())) / 2,
        -(dt * state.omega_O.z()) / 2 - (dt * (state.omega_O.z() - dt * state.omega_O.x() * state.omega_O.y())) / 2, 1 - (dt * (dt * state.omega_O.x() * state.omega_O.x() + dt * state.omega_O.z() * state.omega_O.z())) / 2, (dt * state.omega_O.x()) / 2 + (dt * (state.omega_O.x() + dt * state.omega_O.y() * state.omega_O.z())) / 2,
        (dt * state.omega_O.y()) / 2 + (dt * (state.omega_O.y() + dt * state.omega_O.x() * state.omega_O.z())) / 2, -(dt * state.omega_O.x()) / 2 - (dt * (state.omega_O.x() - dt * state.omega_O.y() * state.omega_O.z())) / 2, 1 - (dt * (dt * state.omega_O.x() * state.omega_O.x() + dt * state.omega_O.y() * state.omega_O.y())) / 2;

    jacobian.block<3, 3>(7, 10) << (dt * (dt * state.v_O.y() * state.omega_O.y() + dt * state.v_O.z() * state.omega_O.z())) / 2, -(dt * state.v_O.z()) / 2 - (dt * (state.v_O.z() - dt * ((981 * (C_WO(0, 0) * C_WO(1, 1) - C_WO(0, 1) * C_WO(1, 0))) / (100 * (C_WO(0, 0) * C_WO(1, 1) * C_WO(2, 2) - C_WO(0, 0) * C_WO(1, 2) * C_WO(2, 1) - C_WO(0, 1) * C_WO(1, 0) * C_WO(2, 2) + C_WO(0, 1) * C_WO(1, 2) * C_WO(2, 0) + C_WO(0, 2) * C_WO(1, 0) * C_WO(2, 1) - C_WO(0, 2) * C_WO(1, 1) * C_WO(2, 0))) - state.v_O.x() * state.omega_O.y() + state.v_O.y() * state.omega_O.x()) + dt * state.v_O.x() * state.omega_O.y())) / 2, (dt * state.v_O.y()) / 2 + (dt * (state.v_O.y() + dt * ((981 * (C_WO(0, 0) * C_WO(1, 2) - C_WO(0, 2) * C_WO(1, 0))) / (100 * (C_WO(0, 0) * C_WO(1, 1) * C_WO(2, 2) - C_WO(0, 0) * C_WO(1, 2) * C_WO(2, 1) - C_WO(0, 1) * C_WO(1, 0) * C_WO(2, 2) + C_WO(0, 1) * C_WO(1, 2) * C_WO(2, 0) + C_WO(0, 2) * C_WO(1, 0) * C_WO(2, 1) - C_WO(0, 2) * C_WO(1, 1) * C_WO(2, 0))) - state.v_O.x() * state.omega_O.z() + state.v_O.z() * state.omega_O.x()) - dt * state.v_O.x() * state.omega_O.z())) / 2,
        (dt * state.v_O.z()) / 2 - (dt * (dt * ((981 * (C_WO(0, 0) * C_WO(1, 1) - C_WO(0, 1) * C_WO(1, 0))) / (100 * (C_WO(0, 0) * C_WO(1, 1) * C_WO(2, 2) - C_WO(0, 0) * C_WO(1, 2) * C_WO(2, 1) - C_WO(0, 1) * C_WO(1, 0) * C_WO(2, 2) + C_WO(0, 1) * C_WO(1, 2) * C_WO(2, 0) + C_WO(0, 2) * C_WO(1, 0) * C_WO(2, 1) - C_WO(0, 2) * C_WO(1, 1) * C_WO(2, 0))) - state.v_O.x() * state.omega_O.y() + state.v_O.y() * state.omega_O.x()) - state.v_O.z() + dt * state.v_O.y() * state.omega_O.x())) / 2, (dt * (dt * state.v_O.x() * state.omega_O.x() + dt * state.v_O.z() * state.omega_O.z())) / 2, -(dt * state.v_O.x()) / 2 - (dt * (state.v_O.x() - dt * ((981 * (C_WO(0, 1) * C_WO(1, 2) - C_WO(0, 2) * C_WO(1, 1))) / (100 * (C_WO(0, 0) * C_WO(1, 1) * C_WO(2, 2) - C_WO(0, 0) * C_WO(1, 2) * C_WO(2, 1) - C_WO(0, 1) * C_WO(1, 0) * C_WO(2, 2) + C_WO(0, 1) * C_WO(1, 2) * C_WO(2, 0) + C_WO(0, 2) * C_WO(1, 0) * C_WO(2, 1) - C_WO(0, 2) * C_WO(1, 1) * C_WO(2, 0))) - state.v_O.y() * state.omega_O.z() + state.v_O.z() * state.omega_O.y()) + dt * state.v_O.y() * state.omega_O.z())) / 2,
        -(dt * state.v_O.y()) / 2 - (dt * (state.v_O.y() + dt * ((981 * (C_WO(0, 0) * C_WO(1, 2) - C_WO(0, 2) * C_WO(1, 0))) / (100 * (C_WO(0, 0) * C_WO(1, 1) * C_WO(2, 2) - C_WO(0, 0) * C_WO(1, 2) * C_WO(2, 1) - C_WO(0, 1) * C_WO(1, 0) * C_WO(2, 2) + C_WO(0, 1) * C_WO(1, 2) * C_WO(2, 0) + C_WO(0, 2) * C_WO(1, 0) * C_WO(2, 1) - C_WO(0, 2) * C_WO(1, 1) * C_WO(2, 0))) - state.v_O.x() * state.omega_O.z() + state.v_O.z() * state.omega_O.x()) + dt * state.v_O.z() * state.omega_O.x())) / 2, (dt * state.v_O.x()) / 2 - (dt * (dt * ((981 * (C_WO(0, 1) * C_WO(1, 2) - C_WO(0, 2) * C_WO(1, 1))) / (100 * (C_WO(0, 0) * C_WO(1, 1) * C_WO(2, 2) - C_WO(0, 0) * C_WO(1, 2) * C_WO(2, 1) - C_WO(0, 1) * C_WO(1, 0) * C_WO(2, 2) + C_WO(0, 1) * C_WO(1, 2) * C_WO(2, 0) + C_WO(0, 2) * C_WO(1, 0) * C_WO(2, 1) - C_WO(0, 2) * C_WO(1, 1) * C_WO(2, 0))) - state.v_O.y() * state.omega_O.z() + state.v_O.z() * state.omega_O.y()) - state.v_O.x() + dt * state.v_O.z() * state.omega_O.y())) / 2, (dt * (dt * state.v_O.x() * state.omega_O.x() + dt * state.v_O.y() * state.omega_O.y())) / 2;

    jacobian.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity();

    return true;
}

bool VisualEKF::stateTransition(const ObjectState &object_state_k_minus_1,
                                ObjectState &object_state_k,
                                const double dt,
                                Eigen::Matrix<double, 13, 13> &jacobian)
{
    ObjectStateDerivative objectStateDerivative_k_minus_1 = calcStateDerivative(object_state_k_minus_1);

    ObjectState delta_x_1;
    delta_x_1.r_W = dt * objectStateDerivative_k_minus_1.r_W_dot;
    delta_x_1.q_WO = dt * objectStateDerivative_k_minus_1.q_WO_dot;
    delta_x_1.v_O = dt * objectStateDerivative_k_minus_1.v_O_dot;
    delta_x_1.omega_O = dt * objectStateDerivative_k_minus_1.omega_O_dot;

    ObjectState state_k_tmp;
    state_k_tmp.r_W = object_state_k_minus_1.r_W + delta_x_1.r_W;
    state_k_tmp.q_WO = object_state_k_minus_1.q_WO + delta_x_1.q_WO;
    state_k_tmp.q_WO.normalize();
    state_k_tmp.v_O = object_state_k_minus_1.v_O + delta_x_1.v_O;
    state_k_tmp.omega_O = object_state_k_minus_1.omega_O + delta_x_1.omega_O;

    ObjectStateDerivative objectStateDerivative_k = calcStateDerivative(state_k_tmp);

    ObjectState delta_x_2;
    delta_x_2.r_W = dt * objectStateDerivative_k.r_W_dot;
    delta_x_2.q_WO = dt * objectStateDerivative_k.q_WO_dot;
    delta_x_2.v_O = dt * objectStateDerivative_k.v_O_dot;
    delta_x_2.omega_O = dt * objectStateDerivative_k.omega_O_dot;

    object_state_k.r_W = object_state_k_minus_1.r_W + 0.5 * (delta_x_1.r_W + delta_x_2.r_W);
    object_state_k.q_WO = object_state_k_minus_1.q_WO + 0.5 * (delta_x_1.q_WO + delta_x_2.q_WO);
    object_state_k.q_WO.normalize();
    object_state_k.v_O = object_state_k_minus_1.v_O + 0.5 * (delta_x_1.v_O + delta_x_2.v_O);
    object_state_k.omega_O = object_state_k_minus_1.omega_O + 0.5 * (delta_x_1.omega_O + delta_x_2.omega_O);

    calcJacobian(object_state_k_minus_1, dt, jacobian);

    return true;
}

bool VisualEKF::predict(const double dt,
                        const ObjectState &fromState,
                        ObjectState &toState)
{
    Eigen::Matrix<double, 13, 13> F;   // Jacobian
    Eigen::Matrix<double, 13, 13> LQL; // Linearised error

    stateTransition(fromState, toState, dt, F);

    LQL.setIdentity();
    LQL.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * sigma_c_r_W * sigma_c_r_W;
    LQL.block<4, 4>(3, 3) = Eigen::Matrix4d::Identity() * dt * sigma_c_q_WO * sigma_c_q_WO;
    LQL.block<3, 3>(7, 7) = Eigen::Matrix3d::Identity() * dt * sigma_c_v_O * sigma_c_v_O;
    LQL.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * dt * sigma_c_omega_O * sigma_c_omega_O;

    P_ = F * P_ * F.transpose() + LQL;

    std::cout << "P predict:" << std::endl << P_.diagonal() << std::endl << std::endl;

    return true;
}

bool VisualEKF::update(const ApriltagMeasurement apriltagMeasurement)
{
    // calculate measurement residual
    Eigen::Matrix<double, 7, 1> y;
    y.segment<3>(0) = apriltagMeasurement.r_W - x_predicted_.r_W;
    y.segment<4>(3) = (apriltagMeasurement.q_WO - x_predicted_.q_WO).coeffs(); // xyzw

    // calculate measurement jacobian
    Eigen::Matrix<double, 7, 13> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<4, 4>(3, 3) = Eigen::Matrix4d::Identity();

    // calculate covariance matrix
    Eigen::Matrix<double, 7, 7> R;
    R.setZero();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_z_r_W * sigma_z_r_W;
    R.block<4, 4>(3, 3) = Eigen::Matrix4d::Identity() * sigma_z_q_WO * sigma_z_q_WO;

    // calculate residual covariance
    Eigen::Matrix<double, 7, 7> S;
    S = H * P_ * H.transpose() + R;

    // calculate kalman gain
    Eigen::Matrix<double, 13, 7> K;
    K = P_ * H.transpose() * S.inverse();

    // calculate state update
    Eigen::Matrix<double, 13, 1> delta;
    delta = K * y;

    // update state
    x_.r_W = x_predicted_.r_W + delta.segment<3>(0);
    x_.q_WO.coeffs() = x_predicted_.q_WO.coeffs() + delta.segment<4>(3);
    x_.q_WO.normalize();
    x_.v_O = x_predicted_.v_O + delta.segment<3>(7);
    x_.omega_O = x_predicted_.omega_O + delta.segment<3>(10);

    // update covariance
    P_ = (Eigen::Matrix<double, 13, 13>::Identity() - K * H) * P_;

    return true;
}