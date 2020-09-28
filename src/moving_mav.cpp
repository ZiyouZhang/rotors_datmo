/**
 * @file moving_mav.cpp
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief The node for controlling the mav movement.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_mav");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started moving mav with camera test.");

  // std_srvs::Empty srv;
  // bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  // unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  // while (i <= 10 && !unpaused) {
  //   ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  //   unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  //   ++i;
  // }

  // if (!unpaused) {
  //   ROS_FATAL("Could not wake up Gazebo.");
  //   return -1;
  // } else {
  //   ROS_INFO("Unpaused the Gazebo simulation.");
  // }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  // ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position;
  double desired_yaw = 0.0;
  double desired_x = 0.0;
  double desired_y = 0.0;
  double desired_z = 0.0;
  int aux = 1.0;

  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {
    // Overwrite defaults if set as node parameters.

    desired_y = -0.5 + 0.1 * (aux % 3);
    desired_z = 0.5 + 0.1 * ((aux % 9) / 3);
    aux++;

    desired_position << desired_x, desired_y, desired_z;

    nh_private.param("x", desired_position.x(), desired_position.x());
    nh_private.param("y", desired_position.y(), desired_position.y());
    nh_private.param("z", desired_position.z(), desired_position.z());
    nh_private.param("yaw", desired_yaw, desired_yaw);

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        desired_position, desired_yaw, &trajectory_msg);

    // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
    //          nh.getNamespace().c_str(), desired_position.x(),
    //          desired_position.y(), desired_position.z());
    trajectory_pub.publish(trajectory_msg);

    ros::spinOnce();

    // Change the set point every 2 seconds
    loop_rate.sleep();
  }

  return 0;
}
