/**
 * @file apriltag_detection.hpp
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief The AprilTag detection node.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef APRILTAG_DETECTION_H_
#define APRILTAG_DETECTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/apriltag_math.h"

#include "visual_ekf.hpp"

class PoseDetector
{
public:
    PoseDetector();
    ~PoseDetector();

    /**
     * @brief The core image processing and dynamic object detection and tracking method.
     * 
     * @param msg Detected image.
     * @param camera_info Camera Information.
     */
    void imageCallBack(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info);

private:
    // ros node handler and publisher
    ros::NodeHandle nh;
    ros::Publisher detectionPub;
    ros::Publisher updatePub;
    ros::Publisher predictionPub;

    // image processing related
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber imageSub;
    image_transport::Publisher imagePub;

    // tf related
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform_WO;
    tf::Quaternion quaternion_WO;

    // apriltag related
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    apriltag_pose_t pose;
    apriltag_detection_info_t info;

    // transformation matrices
    Eigen::Matrix4d T_WO;
    Eigen::Matrix4d T_TO;

    // the ekf estimator and related state topics
    VisualEKF visualEKF;
    nav_msgs::Odometry predicted_state;
    nav_msgs::Odometry updated_state;
    nav_msgs::Odometry detected_state;
    nav_msgs::Odometry lastDetectedState;
    
    // last measurement time, stored to calculate time difference
    double lastMeasurementTime;

    // helper bool values for proper state initialisation
    bool firstDetection = true;
    bool detectionInitialised = false;
    bool skipOnce = true;
};

#endif // APRILTAG_DETECTION_H_