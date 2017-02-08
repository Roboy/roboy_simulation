#pragma once

// std
#include <cstdlib>
#include <iostream>
#include <thread>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
// ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
// ros messages
#include <std_msgs/Int32.h>
// common definitions
#include "common_utilities/CommonDefinitions.h"

using namespace gazebo;
using namespace std;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

class SimulationControl {
public:
    SimulationControl();
    SimulationControl(SimulationControl const&) = delete;
    void operator=(SimulationControl const&) = delete;

    /**
     * Returns a reference to the singleton instance
     * @return reference to the instance
     */
    static SimulationControl &getInstance();

    /**
     * loads the world with the name worldName
     * @param worldName name of the world, e.g. "worlds/empty.world"
     * @return a pointer to the initialized world
     */
    physics::WorldPtr loadWorld(string worldName);

    /**
     * loads the given model in to the world
     * @param world pointer to world to load the model into
     * @param modelName name of the model, e.g. "model://legs_with_muscles_simplified"
     * @return pointer to the loaded model, returns nullptr if load was unsuccessful
     */
    physics::ModelPtr loadModel(physics::WorldPtr world, string modelName);

    /**
     * simulates the given world for a couple of iterations
     * @param world pointer to world
     */
    void simulate(physics::WorldPtr world);

    /**
     * Returns a reference to the rosbag object
     * @return reference to the rosbag
     */
    rosbag::Bag &getRosbag();

    /**
     * Tells whether simulation control is currently recording to a rosbag file
     * @return rosbag recording status
     */
    bool isRecording();

private:
    bool resetWorld(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void simulationControl(const std_msgs::Int32::ConstPtr &msg);

    void initializeInterActiveMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                      physics::ModelPtr model, int roboyID);

    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    ros::Subscriber sim_control_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    bool paused = false;
    bool slow_motion = false;

    rosbag::Bag rosbag;
    string rosbag_filename = "record.bag";
    bool recording = false;
};
