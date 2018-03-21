#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <memory>
#include <map>
//RVIZ
#include "../../common_utilities/include/common_utilities/rviz_visualization.hpp"//needed to visualize stuff in rviz
//ros and more
#include <ros/ros.h>

// ros messages
#include <std_msgs/Int32.h>
#include "roboy_communication_middleware/Pose.h"
#include "roboy_communication_simulation/ExternalForce.h" //needed to receive and process (display) forces
//TODO: simulation control files: for now not sure if needed
#include "roboy_simulation/helperClasses.hpp"
#include "roboy_simulation/simulationControl.hpp"

namespace gazebo
{
    class SimulateRoboyInRViz : public ModelPlugin, public rviz_visualization
    {
    public:
        SimulateRoboyInRViz();
        ~SimulateRoboyInRViz();
        /**
         * Loads the model and creates topics for every revolute joint and subsribes to them
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        /**
         * Is called every gazebo update frame. Makes the model stationary and sets the joint angles of the model
         * to the joint angles in the vector
         */
        void OnUpdate(const common::UpdateInfo &_info);

        /**
         * Reference to world. Not needed for now, maybe later when additional models (such as hand  meshes) need
         * to be added.
         */
        vector<physics::WorldPtr> world;
    private:

        /** Sends two messages: one for RViz, one for the other recipients (Unity )
         */
        void publishPose();

        /**
         * Displays external force from message in RViz
         * @param msg Contains force
         */
        void ApplyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg);

        /**
         * Reference to model to which this plugin belongs
         */
        physics::ModelPtr model;

        /**
         * Binded event for the OnUpdate function
         */
        event::ConnectionPtr updateConnection;
        ros::NodeHandlePtr nh;
        boost::shared_ptr<ros::AsyncSpinner> spinner;

        /**
         * List of all subsribers and publishers
         * *rviz publishers handled using rviz_visualization class*
         */
        ros::Publisher pose_pub;
	    ros::Subscriber external_force_sub;


        math::Pose initPose;
	    int hipID = 0;
    };
}
