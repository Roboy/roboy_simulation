#pragma once
// std
#include <cstdlib>
#include <iostream>
#include <deque>
// ros
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <visualization_msgs/Marker.h>
#include <transmission_interface/transmission_parser.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
// muscle plugin
#include "roboy_simulation/muscle/IMuscle.hpp"
// ros messages
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include "roboy_simulation/MotorControl.h"
#include "roboy_simulation/PIDControl.h"
#include "roboy_simulation/ModelViz.hpp"
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>

using namespace gazebo;
using namespace std;

class ModelController : public gazebo::ModelPlugin, public ModelViz{
public:
    /** Constructor */
    ModelController();
    /** Destructor */
    ~ModelController();

    /**
     * Overloaded Gazebo entry point
     * @param parent model pointer
     * @param sdf element
     */
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    /** Called at each sim step */
    void Update();

    /**
     * Read from Simulation
     * @param time current time
     * @param period period since last read
     */
    void readSim(ros::Time time, ros::Duration period);
    void readSim(gazebo::common::Time, gazebo::common::Time);

    /** Write to Simulation
     * @param time current time
     * @param period period since last read
     */
    void writeSim(ros::Time time, ros::Duration period);
    void writeSim(gazebo::common::Time, gazebo::common::Time);

    /** Called on world reset */
    void Reset();

    /** Calculates the Muscles Forces */
    void updateMuscleForces();

    /** calculates the Muscle Activities */
    void updateMuscleActivities();

    /** This function parses a sdf string for myoMuscle parameters
     * @param sdf string
     * @param myoMuscles will be populated with the paramaters
     * @return success
     */
    bool parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo>& myoMuscles);

    /** Publishes the roboyID of this instantiation */
    void publishID();

    /**
     *   Callback for motor commands
     *   @param msg
    */
    void MotorCommand(const roboy_communication_middleware::MotorCommand::ConstPtr &msg);

private:
    void MotorStatusPublisher();
    static int roboyID_generator;
    int roboyID = 0;
    ros::NodeHandlePtr nh;
    ros::Subscriber  motorCommand_sub, pid_control_sub;
    ros::Publisher visualizeTendon_pub, roboyID_pub, motorStatus_pub;
    ros::ServiceServer roboyID_srv;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    bool motor_status_publishing = true;
    boost::shared_ptr<boost::thread> motor_status_publisher;

    bool e_stop_active, last_e_stop_active;

    // Timing
    gazebo::common::Time gz_time_now;
    gazebo::common::Time gz_period;
    gazebo::common::Time gz_last;
    gazebo::common::Time gz_last_write_sim_time_ros;

    ros::Duration control_period;
    ros::Time last_update_sim_time_ros;
    ros::Time last_write_sim_time_ros;

    // Pointer to the model
    gazebo::physics::ModelPtr parent_model;
    sdf::ElementPtr sdf;

    vector<string> link_names;
    vector<string> joint_names;
    map<string,vector<uint>> muscles_spanning_joint;

    double gazebo_max_step_size = 0.003;

    double initial_contact_time[2];
    bool initial_contact[2];
    math::Vector3 initial_contact_pos[2];
    double v_base = 0;

    // target features
    map<string,math::Quaternion> Q;
    map<string,math::Vector3> P;
    map<string,math::Vector3> v;
    map<string,math::Vector3> omega;
    // target force torque
    map<string,math::Vector3> F;
    map<string,math::Vector3> T;
    map<string,double> tau;
    map<string,double> F_tilde;
    map<string,deque<double>> activity;
    map<string,double> feedback;
    map<string,double> a;

   double *cmd, *pos, *vel, *eff;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection;

    // Strings
    string robot_namespace, robot_description, robot_name;

    uint numberOfMyoMuscles;

    boost::shared_ptr<pluginlib::ClassLoader<roboy_simulation::IMuscle>> class_loader;
    vector<boost::shared_ptr<roboy_simulation::IMuscle>> sim_muscles;
    vector<roboy_simulation::MyoMuscleInfo> myoMuscles; 

    map<string, double> desiredAngles;
    //Mapping of joint's name and its own pid
    map<string, gazebo::common::PID> jointPIDs;

    gazebo::common::Time currentTime;
    gazebo::common::Time previousTime;
    gazebo::common::Time deltaTime;
    bool firstLoop = true;

    double outputMax = 500;
    double outputMin = -500;
};
