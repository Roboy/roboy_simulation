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
// additional

#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/physics/Model.hh"
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
#include "gazebo/common/Plugin.hh"


using namespace gazebo;
using namespace std;

class VRRoboy2 : public SimulationControl { //,public  rviz_visualization{
public:
    VRRoboy2();
    ~VRRoboy2();
    /**
     * initializes numberOfWorlds worlds and populates them with the legs
     * @param numberOfWorlds
     */
    void initializeWorlds(uint numberOfWorlds);

    void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr sdf);
    /**
     * Publishes the pose of a roboy
     * @param modelNr the roboy id
     */
    void publishPose(uint modelNr);

    /**
     * Publishes the motor state of a roboy
     * @param modelNr the roboy id
     */
    void publishMotorStates(uint modelNr);
    /**
     * Applies the external force to all roboys
     * @param msg the external force
     */
    void applyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg);

    /**
     * Reference to all created worlds
     */
    vector<physics::WorldPtr> world;

private:

    ros::NodeHandlePtr nh;
    ros::Subscriber pose_sub, external_force_sub;
    ros::Publisher  pose_pub, muscle_state_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    vector<physics::ModelPtr> model; // list of models
};

int main(int _argc, char **_argv);
