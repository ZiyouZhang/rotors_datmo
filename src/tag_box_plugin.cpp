/**
 * @file tag_box_plugin.cpp
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief Tagged box plugin source file.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/console.h>
#include <geometry_msgs/Pose.h>

#include "tag_box_plugin.hpp"

namespace gazebo
{
    void TagBoxPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;
        msg.header.frame_id = model->GetName();

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&TagBoxPlugin::OnUpdate, this));

        posePub = nh.advertise<nav_msgs::Odometry>("/tag_box_pose_ground_truth", 1);

        model->SetGravityMode(false);
        model->SetEnabled(false);
    }

    void TagBoxPlugin::OnUpdate()
    {
        ros::Time currentTime = ros::Time::now();

        if (!pose_initialised && currentTime.toSec() > 3) {
            model->SetWorldPose(ignition::math::Pose3d(5.0, -0.5, 0.3, 0.0, 0.0, 0.0));
            pose_initialised = true;
        }
        
        if (!vel_initialised && nh.hasParam("/start_motion"))
        {
            bool startMotion = false;
            nh.getParam("/start_motion", startMotion);
            if (startMotion)
            {
                model->SetGravityMode(true);
                model->SetLinearVel(ignition::math::Vector3d(-4.0, 0.5, 4.0));
                model->SetAngularVel(ignition::math::Vector3d(1.0, 0.0, 0.0));
                vel_initialised = true;
            }
        }

        // Publish pose message
        msg.header.stamp = currentTime;
        msg.pose.pose.position.x = model->WorldPose().Pos().X();
        msg.pose.pose.position.y = model->WorldPose().Pos().Y();
        msg.pose.pose.position.z = model->WorldPose().Pos().Z();
        msg.pose.pose.orientation.x = model->WorldPose().Rot().X();
        msg.pose.pose.orientation.y = model->WorldPose().Rot().Y();
        msg.pose.pose.orientation.z = model->WorldPose().Rot().Z();
        msg.pose.pose.orientation.w = model->WorldPose().Rot().W();
        msg.twist.twist.linear.x = model->RelativeLinearVel()[0];
        msg.twist.twist.linear.y = model->RelativeLinearVel()[1];
        msg.twist.twist.linear.z = model->RelativeLinearVel()[2];
        msg.twist.twist.angular.x = model->RelativeAngularVel()[0];
        msg.twist.twist.angular.y = model->RelativeAngularVel()[1];
        msg.twist.twist.angular.z = model->RelativeAngularVel()[2];
        posePub.publish(msg);

        // Broadcast tf
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(model->WorldPose().Pos().X(),
                                        model->WorldPose().Pos().Y(),
                                        model->WorldPose().Pos().Z()));
        transform.setRotation(tf::Quaternion(model->WorldPose().Rot().X(),
                                             model->WorldPose().Rot().Y(),
                                             model->WorldPose().Rot().Z(),
                                             model->WorldPose().Rot().W()));
        br.sendTransform(tf::StampedTransform(transform, currentTime, "world", model->GetName()));
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(TagBoxPlugin)
} // namespace gazebo