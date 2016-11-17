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
// ros messages
#include <std_msgs/Int32.h>
#include "common_utilities/Pose.h"
// common definitions
#include "common_utilities/CommonDefinitions.h"
#include "roboy_simulation/helperClasses.hpp"
#include "roboy_simulation/simulationControl.hpp"

using namespace gazebo;
using namespace std;

class VRRoboy : public SimulationControl{
public:
    VRRoboy();
    ~VRRoboy();
    /**
     * initializes numberOfWorlds worlds and populates them with the legs
     * @param numberOfWorlds
     */
    void initializeWorlds(uint numberOfWorlds);
    void publishPose(uint modelNr);
    vector<physics::WorldPtr> world;
private:
    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    ros::Subscriber sim_control_sub;
    ros::Publisher pose_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    vector<physics::ModelPtr> model;
};

int main(int _argc, char **_argv);