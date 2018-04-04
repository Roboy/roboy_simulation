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
#include <geometry_msgs/Pose.h>
#include "roboy_communication_middleware/Pose.h"
#include "roboy_communication_middleware/MuscleState.h"
#include "roboy_communication_simulation/ExternalForce.h"
// common definitions
#include "common_utilities/CommonDefinitions.h"
#include "roboy_simulation/helperClasses.hpp"
#include "roboy_simulation/simulationControl.hpp"

using namespace gazebo;
using namespace std;

class VRRoboy : public SimulationControl{
public:
    /**
     * Constructor: sets up connection to ROS, initializes world and loads model
     */
    VRRoboy();

    ~VRRoboy();

    /**
     * Publishes the current pose of the roboy of the model
     */
    void publishPose();

    /**
     * Publishes random motor values
     */
    void publishRandomMotorStates();

    /**
     * Applies force to initially instantiated ROboy model. Expects coordinates in gazebo's coordinate system,
     * @param msg Message containing link and coordinates in local space of the given link
     */
    void applyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg);

    /**
     * pointer to created world
     */
    physics::WorldPtr world;

private:
    //connection related
    transport::NodePtr node;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::NodeHandlePtr nh;

    //publisher & subscriber
    ros::Subscriber external_force_sub;
    ros::Publisher  pose_pub, muscle_state_pub;
    /**
     * model loaded into the world (since messages cannot distinguish models, multiple models in one world useless-> one model)
     */
    physics::ModelPtr model;
};

int main(int _argc, char **_argv);