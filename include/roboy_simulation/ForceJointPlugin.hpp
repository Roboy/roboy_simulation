#pragma once
#include <ros/ros.h>
#include <roboy_communication_middleware/JointCommandRevolute.h>
#include <roboy_communication_middleware/JointCommandRevolute2.h>


#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>

#include "roboy_communication_middleware/Pose.h"
#include "roboy_communication_middleware/DarkRoomSensor.h"

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
         * Is called every gazebo update frame. Makes the model stationary and sets the joint angles of the model
         * to the joint angles in the vector
         */
        void OnUpdate(const common::UpdateInfo &_info);
    private:
        void publishPose();
        void JointCommandRevolute(const roboy_communication_middleware::JointCommandRevoluteConstPtr &_msg);
	void JointCommandRevolute2(const roboy_communication_middleware::JointCommandRevolute2ConstPtr &_msg);
	void DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensorConstPtr &_msg);
        physics::ModelPtr model;
        /**
         * Binded event for the OnUpdate function
         */
        event::ConnectionPtr updateConnection;
        ros::NodeHandlePtr nh;
        boost::shared_ptr<ros::AsyncSpinner> spinner;
        /**
         * List of all joint subsribers
         */
        ros::Publisher pose_pub;
	ros::Subscriber hip_sub;
        ros::Subscriber jointCommandRevolute_sub;
	ros::Subscriber jointCommandRevolute2_sub;
        std::list<std::string> jointsRevolute;
	std::list<std::string> jointsRevolute2;
        /**
         * Map for storing the current joint angles, is updated in OnRosMsg
         */
        std::map<std::string, double> jointAnglesRevolute;
        std::map<std::string, double[2]> jointAnglesRevolute2;
        math::Pose initPose;
	int hipID = 0;
    };
}
