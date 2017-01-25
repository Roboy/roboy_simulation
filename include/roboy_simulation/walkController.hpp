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
#include "roboy_simulation/Tendon.h"
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/ForceTorque.h"
#include "roboy_simulation/LegState.h"
#include "roboy_simulation/ControllerParameters.h"
#include "roboy_simulation/UpdateControllerParameters.h"
#include "roboy_simulation/Energies.h"
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Trajectory.h"
#include "common_utilities/RoboyState.h"
#include "roboy_simulation/Abortion.h"
#include "roboy_simulation/MotorControl.h"
#include "roboy_simulation/IMU.h"
#include "roboy_simulation/Joint.h"
#include "roboy_simulation/BodyPart.h"
#include "roboy_simulation/COM.h"

#include "roboy_simulation/walkVisualization.hpp"
#include "roboy_simulation/helperClasses.hpp"
#include "roboy_simulation/controllerParameters.hpp"

using namespace gazebo;
using namespace std;
//using namespace libcmaes;

static const char * FOOT[] = { "foot_left", "foot_right" };

static const char * LEG_STATE_STRING[] = { "Stance", "Lift_off", "Swing", "Stance_Preparation" };

static const char * LEG_NAMES_STRING[] = { "left leg", "right leg" };

static const uint ACCEL_WIN_SIZE = 20;

class WalkController : public gazebo::ModelPlugin, public WalkVisualization{
public:
    /** Constructor */
    WalkController();
    /** Destructor */
    ~WalkController();

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

    /** Write to Simulation
     * @param time current time
     * @param period period since last read
     */
    void writeSim(ros::Time time, ros::Duration period);

    /** Called on world reset */
    void Reset();

    /**
     * Callback function for force torque sensors, defines the state of eaach leg
     * @param msg values from force torque sensor
     */
    void finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg);

    /**
     * Function used by finite_state_machine, defines the state order
     * @param s current state
     * @return next state
     */
    LEG_STATE NextState(LEG_STATE s);

    /**
     * Function for checking which leg is in a certain state
     * @param s query state
     * @return leg
     */
    LEG getLegInState(LEG_STATE s);

    /**
     * Calculates the COM position or velocity
     * @param type POSITION or VELOCITY
     * @param COM is filled with the COM position/velocity
     */
    void calculateCOM(int type, math::Vector3 &COM);

    /**
     * Publishes the estimated position of COM based on the neural network
     * interpretation of sensor data
     */
    void publishEstimatedCOM();

    /**
     * Fetches the accelerations and the positions of the IMUs and
     * publishes them on a topic
     */
    void publishIMUs();

    /** updates foot displacements and velocity of each foot wrt to hip orientation */
    void updateFootDisplacementAndVelocity();

    /** Calculates the target features of every link wrt to current leg state */
    void updateTargetFeatures();

    /** Calculates the Muscles Forces */
    void updateMuscleForces();

    /** calculates the Muscle Activities */
    void updateMuscleActivities();

    /** Calculates the energies */
    void updateEnergies();

    /** checks if any of the abortion criteria is reached
     * @return true (abort), false (continue)
     * */
    bool checkAbort();

    /** Emergency stop callback */
    void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

    /** This function parses a sdf string for myoMuscle parameters
     * @param sdf string
     * @param myoMuscles will be populated with the paramaters
     * @return success
     */
    bool parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo>& myoMuscles);

    /** Publishes the roboyID of this instantiation */
    void publishID();

    /** Callback for toggle of walkControl
     * @param msg containing the trigger (true/false)
     * */
    void toggleWalkController(const std_msgs::Bool::ConstPtr &msg);

    /** Callback for manual motor control
     * @param msg contains vector with voltage values for every motor
     * */
    void motorControl(const roboy_simulation::MotorControl::ConstPtr &msg);


    /**
     * Service for update of control parameters, typically sent by walkTrainer
     * @param req with new control parameters
     * @param res not used
     * @return success
     */
    bool updateControllerParameters(roboy_simulation::UpdateControllerParameters::Request  &req,
                                    roboy_simulation::UpdateControllerParameters::Response &res);
    /**
     * Service for retrieving current energies, typically sent by walkTrainer
     * @param req not used
     * @param res energies
     * @return success
     */
    bool energiesService(roboy_simulation::Energies::Request  &req,
                         roboy_simulation::Energies::Response &res);

    void publishJoints ();

    void publishCOMmsg ();


private:
    static int roboyID_generator;
    int roboyID = 0;
    ros::NodeHandlePtr nh;
    ros::Subscriber force_torque_ankle_left_sub, force_torque_ankle_right_sub, motor_control_sub,
            steer_recording_sub, record_sub, init_sub, toggle_walk_controller_sub, e_stop_sub;
    ros::Publisher visualizeTendon_pub, roboyID_pub, abort_pub, imu_pub, joint_pub, body_pub, COM_pub;
    ros::ServiceServer roboyID_srv, control_parameters_srv, energies_srv;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    bool e_stop_active, last_e_stop_active;

    // Timing
    gazebo::common::Time gz_time_now;
    ros::Duration control_period;
    ros::Time last_update_sim_time_ros;
    ros::Time last_write_sim_time_ros;

    // Pointer to the model
    gazebo::physics::ModelPtr parent_model;
    sdf::ElementPtr sdf;

    vector<string> link_names;
    vector<string> joint_names;
    map<string,vector<uint>> muscles_spanning_joint;

    // Acceleration "windows" for calculating the filtered accelerations to be published as IMU sensor data.
    // This is done to make the IMU data less noisy.
    // key: link name, value: array of accelerations
    map<string, vector<math::Vector3>> acceleration_windows;

    // Gaussian kernel for filtering the IMU data
    vector<double> gaussian_kernel;

    // Returns the filtered linear acceleration for a link
    math::Vector3 getFilteredLinearAcceleration(const physics::LinkPtr link);

    double gazebo_max_step_size = 0.003;

    LEG_STATE leg_state[2];

    bool control = false;

    ControllerParameters params;

    double v_forward = 1.0;
    double psi_heading = 0.0;

    math::Vector3 foot_sole[2], foot_sole_global[2], d_foot_pos[2], d_foot_vel[2];

    double initial_contact_time[2];
    bool initial_contact[2];
    math::Vector3 initial_contact_pos[2];
    double v_base = 0;

    // coordinate systems
    CoordSys hip_CS;

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

    // energies
    double E_speed = 0, E_headori = 0, E_headvel = 0, E_slide = 0, E_effort = 0;
    double E_speed_int = 0, E_headori_int = 0, E_headvel_int = 0, E_slide_int = 0, E_effort_int = 0;
    // weights
    double w_speed = 100.0, w_headori = 10.0, w_headvel = 10.0, w_slide = 10.0, w_effort = 0.1;
    // thresholds
    double H_speed = 0.1, H_headori = 0.2, H_headvel = 0.3, H_slide = 0.2, H_effort = 0;

    math::Vector3 center_of_mass[2], initial_center_of_mass_height;
    double v_COM;

    double *cmd, *pos, *vel, *eff;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection;

    // Strings
    string robot_namespace, robot_description;

    uint numberOfMyoMuscles;

    boost::shared_ptr<pluginlib::ClassLoader<roboy_simulation::IMuscle>> class_loader;
    vector<boost::shared_ptr<roboy_simulation::IMuscle>> sim_muscles;
    vector<roboy_simulation::MyoMuscleInfo> myoMuscles;
};
