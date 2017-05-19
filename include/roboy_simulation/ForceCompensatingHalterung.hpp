// ros
#include <ros/ros.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
// ros messages
#include "roboy_communication_simulation/ForceTorque.h"
#include "std_msgs/Int32.h"

using namespace gazebo;

class ForceCompensatingHalterung : public gazebo::ModelPlugin{
public:
    /** Constructor */
    ForceCompensatingHalterung();
    /** Destructor */
    ~ForceCompensatingHalterung();
    /**
     * Overloaded Gazebo entry point
     * @param parent model pointer
     * @param sdf element
     */
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    /** Called at each sim step */
    void Update();

private:
    void updateID(const std_msgs::Int32::ConstPtr &msg);
    void updateForce(const roboy_communication_simulation::ForceTorque::ConstPtr &msg);

    int roboyID = 0;
    ros::NodeHandlePtr nh;
    ros::Subscriber force_torque_halterung_sub, roboyID_sub;
    math::Vector3 compensation_force;

    // Pointer to the model
    gazebo::physics::ModelPtr parent_model;
    sdf::ElementPtr sdf;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection;

    gazebo::common::Time gz_time_now;
};