#pragma once
#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>

#include "roboy_communication_middleware/Pose.h"
#include <std_srvs/Trigger.h>

namespace gazebo
{
    class ForceLinkPlugin : public ModelPlugin
    {
    public:
        ForceLinkPlugin();
        ~ForceLinkPlugin();
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
        void PoseCommand(const roboy_communication_middleware::PoseConstPtr &_msg);
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
        ros::Subscriber pose_sub;
        std::list<std::string> linkNames;
        /**
         * Map for storing the current joint angles, is updated in OnRosMsg
         */
        std::map<std::string, math::Pose> linkPoses;
        math::Pose initPose;
    };
}
