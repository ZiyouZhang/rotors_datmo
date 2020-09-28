/**
 * @file visual_ekf.hpp
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief Design of visual-based EKF estimation of dynamic object state.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef VISUAL_EKF_HPP_
#define VISUAL_EKF_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>

struct ObjectState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double mass;
    double timestamp;                            // Time stamp in seconds.
    Eigen::Vector3d r_W;                         // The position relative to the W frame.
    Eigen::Quaterniond q_WO;                     // The quaternion of rotation W-O.
    Eigen::Vector3d v_O;                         // The velocity expressed in object frame.
    Eigen::Vector3d omega_O;                     // The angular velocity expressed in object frame.
    Eigen::Matrix<double, 3, 3> inertia;         //The moment of inertia.
    Eigen::Matrix<double, 3, 3> inertia_inverse; // The inverse of moment of inertia, stored for better efficiency.

    ObjectState()
    {
        inertia << 0.5 / 12, 0, 0,
            0, 0.5 / 12, 0,
            0, 0, 0.5 / 12;

        inertia_inverse = inertia.inverse();
    }
};

struct ObjectStateDerivative
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;            // Time stamp in seconds.
    Eigen::Vector3d r_W_dot;     // The position relative to the W frame.
    Eigen::Quaterniond q_WO_dot; // The quaternion of rotation W-O.
    Eigen::Vector3d v_O_dot;     // The velocity expressed in object frame.
    Eigen::Vector3d omega_O_dot; // The angular velocity expressed in object frame.
};

struct ApriltagMeasurement
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    double timestamp;        // Time stamp in seconds.
    Eigen::Vector3d r_W;     // The position relative to the W frame.
    Eigen::Quaterniond q_WO; // The quaternion of rotation W-O.
};

class VisualEKF
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VisualEKF();
    ~VisualEKF();

    /**
     * @brief The initialisation state.
     * 
     * @return true if initialised
     * @return false if not initialised
     */
    bool isInitialsed();

    /**
     * @brief Initialise the EKF with the initial state.
     * 
     * @param initialState Initial object state.
     * @return true if successfully executed.
     */
    bool initialise(const ObjectState initialState);

    /**
     * @brief The predict step in EKF.
     * 
     * @param dt The time difference.
     * @param fromState The previous state.
     * @param toState The predicted state.
     * @return true if successfully executed.
     */
    bool predict(const double dt,
                 const ObjectState &fromState,
                 ObjectState &toState);

    /**
     * @brief The update step in EKF.
     * 
     * @param apriltagMeasurement The apriltag measurement (pose and oritation of the object).
     * @return true if successfully executed.
     */
    bool update(const ApriltagMeasurement apriltagMeasurement);

    /**
     * @brief The state transition using trapezoidal numerical integration.
     * 
     * @param object_state_k_minues_1 The state at previous step.
     * @param object_state_1 The resulting state after trapezoidal numerical integration.
     * @param dt The time difference.
     * @param jacobian The jacobian matrix to be calculated.
     * @return true if successfully executed.
     */
    bool stateTransition(const ObjectState &object_state_k_minues_1,
                         ObjectState &object_state_1,
                         const double dt,
                         Eigen::Matrix<double, 13, 13> &jacobian);

    /**
     * @brief Calculate the derivatives based on the motion model.
     * 
     * @param state The object state.
     * @return ObjectStateDerivative The object state derivatives.
     */
    ObjectStateDerivative calcStateDerivative(const ObjectState &state);

    /**
     * @brief Calculate the jacobian matrix.
     * 
     * @param state The object state.
     * @param dt The time difference.
     * @param jacobian The jacobian matrix to be calculated.
     * @return true if successfully executed.
     */
    bool calcJacobian(const ObjectState &state,
                      const double &dt,
                      Eigen::Matrix<double, 13, 13> &jacobian);

private:
    // The object states.
    ObjectState x_;
    ObjectState x_predicted_;
    ObjectState x_temp_;

    // The initialsation information.
    bool initialised = false;

    // The error matrix and jacobian matrix.
    Eigen::Matrix<double, 13, 13> P_;
    Eigen::Matrix<double, 13, 13> jacobian;

    // Process noise params.
    double sigma_c_r_W = 0.2;     // m, location error
    double sigma_c_q_WO = 0.4;    // N/A, for quoternion error
    double sigma_c_v_O = 3;     // m/s, velocity error
    double sigma_c_omega_O = 10; // rad/s, amgular velocity error

    //Measurement noise params.
    double sigma_z_r_W = 0.02;  // m, pose measurement error
    double sigma_z_q_WO = 0.02; // N/A, quaternion measurement error

    friend class PoseDetector;
};

#endif // VISUAL_EKF_HPP_