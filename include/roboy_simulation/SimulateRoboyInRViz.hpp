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

    private:

        /** Sends two messages: one for RViz, one for the other recipients (e.g. Unity)
         */
        void publishPose();

        /**
         * Displays external force from message in RViz
         * SEE IMPLEMENTATION FOR MORE DETAILS
         * @param msg Contains force, pos & dir in gazebo coordinate space
         */
        void ApplyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg);

        /**
         * Prints simulation status: how many pose-msgs were published, how many force msgs received
         */
        void PrintStats();

        /**
         * Reference to model to which this plugin belongs
         */
        physics::ModelPtr model;

        //connection params
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

        /** TIME Related variables (For msg / frame counts)*/

        /**
         * CAP FOR FPS - ADJUST IF NECESSARY
         * actual fps <= FRAMESPERSEC
         */
        int FRAMESPERSEC = 80;
        /**
         * Counter for seconds since simulation start - for PrintStats()
         */
        std::time_t seconds = 0;
        /**
         * counter of #pose_msgs sent in one second (reset for reach second) - for PrintStats()
         */
        int PoseCounter = 0;
        /**
         * counter of #force_msgs sent in one second (reset for reach second) -  for PrintStats()
         */
        int ForceCounter = 0;
        /**
         * Time of last frame
         */
        std::chrono::high_resolution_clock::time_point prevTime;

    };
}
