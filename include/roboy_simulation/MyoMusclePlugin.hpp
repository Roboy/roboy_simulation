#pragma once

#include <cstdlib>
#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <visualization_msgs/Marker.h>
#include <transmission_interface/transmission_parser.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include "roboy_simulation/muscle/IMuscle.hpp"
#include "roboy_simulation/MyoMuscleVisualization.hpp"
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <roboy_communication_middleware/ControlMode.h>
#include <chrono>
#include <common_utilities/CommonDefinitions.h>
#include <mutex>
#include <std_srvs/SetBool.h>

using namespace gazebo;
using namespace std;
using namespace chrono;

class MyoMusclePlugin : public gazebo::ModelPlugin, public MyoMuscleVisualization{
public:
    /** Constructor */
    MyoMusclePlugin();
    /** Destructor */
    ~MyoMusclePlugin();

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
private:
    /**
     *   Callback for motor commands
     *   @param msg
    */
    void MotorCommand(const roboy_communication_middleware::MotorCommand::ConstPtr &msg);
    void MotorStatusPublisher();
    bool MotorConfigService(roboy_communication_middleware::MotorConfigService::Request &req,
                                             roboy_communication_middleware::MotorConfigService::Response &res);
    bool ControlModeService(roboy_communication_middleware::ControlMode::Request &req,
                                             roboy_communication_middleware::ControlMode::Response &res);
    bool EmergencyStopService(std_srvs::SetBool::Request &req,
                                               std_srvs::SetBool::Response &res);

    bool emergency_stop = false;

    static int roboyID_generator;
    int roboyID = 0;
    ros::NodeHandlePtr nh;
    ros::Subscriber  motorCommand_sub, pid_control_sub;
    ros::Publisher visualizeTendon_pub, roboyID_pub, motorStatus_pub;
    ros::ServiceServer roboyID_srv, motorConfig_srv,  controlMode_srv, emergencyStop_srv;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    bool motor_status_publishing = true;
    boost::shared_ptr<boost::thread> motor_status_publisher = nullptr;

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

    high_resolution_clock::time_point t0,t1;
    mutex mux;
};