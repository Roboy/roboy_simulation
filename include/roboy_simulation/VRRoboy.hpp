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
#include "common_utilities/Pose.h"
#include "common_utilities/MuscleState.h"
#include "common_utilities/ExternalForce.h"
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
    /**
     * Publishes the pose of a roboy
     * @param modelNr the roboy id
     */
    void publishPose(uint modelNr);
    void publishTestPose( const geometry_msgs::Pose::ConstPtr& msg );
    /**
     * Publishes the motor state of a roboy
     * @param modelNr the roboy id
     */
    void publishMotorStates(uint modelNr);
    /**
     * Applies the external force to all roboys
     * @param msg the external force
     */
    void applyExternalForce(const common_utilities::ExternalForce::ConstPtr &msg);
    vector<physics::WorldPtr> world;
    bool apply_external_force = false;
    int32_t duration_in_milliseconds = 0;
private:
    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    ros::Subscriber sim_control_sub, pose_sub, external_force_sub;
    ros::Publisher marker_visualization_pub, pose_pub, muscle_state_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    vector<physics::ModelPtr> model;
};

int main(int _argc, char **_argv);