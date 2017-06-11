#pragma once
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>

#include "roboy_communication_middleware/Pose.h"

namespace gazebo
{
    class ForceJointPlugin : public ModelPlugin
    {
    public:
        ForceJointPlugin();
        ~ForceJointPlugin();
        /**
         * Loads the model and creates topics for every revolute joint and subsribes to them
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        /**
         * Updates the joint angles vector
         */
        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, std::string jointName);
        /**
         * Is called every gazebo update frame. Makes the model stationary and sets the joint angles of the model
         * to the joint angles in the vector
         */
        void OnUpdate(const common::UpdateInfo &_info);
        void publishPose();
    private:
        void QueueThread();
        physics::ModelPtr model;
        /**
         * Binded event for the OnUpdate function
         */
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> nh;
        /**
         * List of all joint subsribers
         */
        std::list<ros::Subscriber> rosSubList;
        ros::Publisher pose_pub;
        std::list<std::string> joints;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        /**
         * Map for storing the current joint angles, is updated in OnRosMsg
         */
        std::map<std::string, double> jointAngles;
        math::Pose initPose;
    };
}
