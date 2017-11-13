#pragma once

#include "IActuator.hpp"
#include "ISee.hpp"
#include "IViaPoints.hpp"
#include "SphericalWrapping.hpp"
#include "CylindricalWrapping.hpp"
#include "MeshWrapping.hpp"
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>
// ros
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
// messages
#include <std_msgs/Float32.h>
// boost
#include <boost/numeric/odeint.hpp>
#include <boost/bind.hpp>
//std
#include <math.h>
#include <map>
#include <stdio.h>
#include <sstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <cmath>
#include <common_utilities/rviz_visualization.hpp>
#include "roboy_simulation/pid.hpp"

enum MUSCLE_TYPE{
    EXTENSOR,
    FLEXOR,
    STABILIZER
};

namespace roboy_simulation {

	using namespace std;
	using namespace boost::numeric::odeint;
	using namespace gazebo;

	struct MyoMuscleInfo{
		string name;
		vector<ViaPointInfo> viaPoints;
		Motor motor;
		Gear gear;
		Spindle spindle;
		SEE see;
		MUSCLE_TYPE muscle_type;
        physics::JointPtr spanningJoint;
	};

	class IMuscle:public rviz_visualization{

	public:
		IMuscle();
		~IMuscle();

        ////////////////////
		/// \brief The Init function.
		///
		/// This function initializes the plugin.
		/// \param[in] myoMuscle contains info about via points, motor, gear, spindle and see of the muscles
		void Init(MyoMuscleInfo &myoMuscle);
		void Update(ros::Time &time, ros::Duration &period );
		string name;
		vector<std::shared_ptr<IViaPoints>> viaPoints;
		double cmd = 0;
		bool pid_control = false;
		int feedback_type = 0;
		
		math::Vector3 momentArm;
        physics::JointPtr spanningJoint = nullptr;
	private:
        ros::NodeHandlePtr nh;
        ros::Publisher muscleForce_pub;
		ros::Publisher seeForce_pub;
		ros::Publisher motorCurrent_pub;
		ros::Publisher spindleAngVel_pub;
		ros::Publisher totalLength_pub;
		ros::Publisher tendonLength_pub;

    public:
        int roboyID;
		ISee see;
		IActuator actuator;

    private:
        IActuator::state_type x;
		//actuatorForce ist the force generated by the motor
		double actuatorForce = 0;
    public:
		//Motorforce is the force getting applied onto the first Viapoint
		double muscleForce = 0;
    private:
		//muscleLength describes the TendonLength from the first Viapoint to the last viapoint. (TendonLength outside the myoMotor)
        double muscleLength = 0;
		// prevMuscleLength is needded to calculate the actual angVel of the motor
		double prevMuscleLength;
		//tendonLength describes the total TendonLength from then Motor until the last viapoint. (TendonLength excluding the tendon coiled up on the motor)
        double tendonLength;
		//initial TendonLength describes the initial total tendonlength 
        double initialTendonLength;
		//actual angVel
		double sim_angVel;
        bool firstUpdate;
		double sinParm = 0;
		double feedback[3] = {0.0, 0.0, 0.0};
		PID musclePID = PID( 24.0, -24.0, 150, 10, 100);

		void setupTopics();
		void publishTopics();
		void initViaPoints( MyoMuscleInfo &myoMuscle );
		void calculateTendonForceProgression();
		void applyMotorCurrent( double &motorCurrent, const double &spindleAngVel );
		void applySpindleAngVel( const double &motorCurrent, double &spindleAngVel );
	};


}
