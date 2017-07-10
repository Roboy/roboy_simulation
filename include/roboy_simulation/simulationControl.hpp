#pragma once

// std
#include <iostream>
#include <thread>
#include <mutex>
// posix
#include <sys/stat.h>
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
#include "roboy_simulation/RecordingControl.h"
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
     * Writes rostopic messages to rosbag if recording is on
     * @param topic rostopic name
     * @param msg the message to be written to the rosbag
     */
    template <typename T>
    void writeRosbagIfRecording(string topic, T &msg) {
        lock_guard<mutex> guard(rosbag_mutex);
        if (recording) {
            rosbag.write(topic.c_str(), ros::Time::now(), msg);
        }
    }

private:
    bool resetWorld(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void simulationControl(const std_msgs::Int32::ConstPtr &msg);

    /**
     * Parses the message sent by clicking "Reset and start recording".
     * Resets the simulation and initiates recording for the given timespan.
     * @param msg The message that contains time for recording
     */
    void recordingControl(const roboy_simulation::RecordingControl::ConstPtr &msg);

    void initializeInterActiveMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                      physics::ModelPtr model, int roboyID);

    /**
     * Opens a new rosbag file for recording. Searches for filenames
     * rosbag_0, rosbag_1, etc. and when a free name is found, open that file.
     */
    void startRecording();

    /**
     * Closes the rosbag file.
     */
    void stopRecording();

    /**
     * Resets start_time and stop_time to 0.
     */
    void resetRecordingTime();

    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    ros::Subscriber sim_control_sub, recording_control_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    bool paused = false;
    bool slow_motion = false;

    rosbag::Bag rosbag;
    const char rosbag_filename_template[14] = "record_%i.bag";
    char rosbag_filename[25];

    // Recording times received from balancing_plugin
    uint32_t start_time = 0;
    uint32_t stop_time = 0;
    bool recording = false;

    // Mutex for accessing variables 'recording' and 'rosbag'.
    // rosbag is opened in this thread but the data is written in another,
    // from WalkController class.
    mutex rosbag_mutex;
};
