#include "roboy_simulation/muscle/IMuscle.hpp"


namespace roboy_simulation {

    IMuscle::IMuscle() : muscleLength(0), tendonLength(0), initialTendonLength(0), firstUpdate(true) {
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "MusclePlugin",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }
        //x[0] = motorcurrent
        //x[1] = sindleAngleVel
        x.resize(2);

        // Setup topics for different motor values
        setupTopics();
    }

    void IMuscle::Init(MyoMuscleInfo &myoMuscle) {
        
        //state initialization
        x[0] = 0.0;
        x[1] = 0.0;
        actuator.motor.voltage = 0.0;
        actuator.spindle.angVel = 0;

        /// Build Linked Viapoint list with corresponding wraping
        initViaPoints( myoMuscle );

        actuator.motor = myoMuscle.motor;
        actuator.gear = myoMuscle.gear;
        actuator.spindle = myoMuscle.spindle;
        see.see = myoMuscle.see;
        name = myoMuscle.name;
        see.see.expansion = 0.0;
        see.see.force = 0.0;
    }

    void IMuscle::Update(ros::Time &time, ros::Duration &period) {
        actuator.motor.voltage = cmd;

        for (int i = 0; i < viaPoints.size(); i++) {
            // absolute position + relative position=actual position of each via point
            viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
                                              viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);
        }

        //update force points and calculate muscle length for each Viapoint
        //muscleLength is set zero and then added up again
        muscleLength = 0;
        for (int i = 0; i < viaPoints.size(); i++) {
            viaPoints[i]->UpdateForcePoints();
            muscleLength += viaPoints[i]->previousSegmentLength;
            
            if (firstUpdate) {
                ROS_INFO("global coordinates are %f %f %f ", viaPoints[i]->globalCoordinates.x,
                         viaPoints[i]->globalCoordinates.y, viaPoints[i]->globalCoordinates.z);
                ROS_INFO("prevForcePoint is %f %f %f ", viaPoints[i]->prevForcePoint.x, viaPoints[i]->prevForcePoint.y,
                         viaPoints[i]->prevForcePoint.z);
                ROS_INFO("nextForcePoint is %f %f %f", viaPoints[i]->nextForcePoint.x, viaPoints[i]->nextForcePoint.y,
                         viaPoints[i]->nextForcePoint.z);
                ROS_INFO("segmentlength is %f ", viaPoints[i]->previousSegmentLength);
            }
        };

        if (firstUpdate) {
            initialTendonLength = tendonLength = muscleLength + see.internalLength;

            ROS_INFO("Calculated musclelength is %f m", muscleLength);
            firstUpdate = false;
        }

        //calculate elastic force
        see.ElasticElementModel( tendonLength, muscleLength );
        see.applyTendonForce( muscleForce, actuator.elasticForce );

        calculateTendonForceProgression();

        // calculate the approximation of gear's efficiency
        actuator.gear.appEfficiency = actuator.EfficiencyApproximation();

        // do 1 step of integration of DiffModel() at current time        
        actuator.stepper.do_step([this](const IActuator::state_type &x, IActuator::state_type &dxdt, const double ) {
            // This lambda function describes the differential model for the simulations of dynamics
            // of a DC motor, a spindle, and a gear box`
            // x[0] - motor electric current
            // x[1] - spindle angular velocity
            double totalIM = actuator.motor.inertiaMoment + actuator.gear.inertiaMoment; // total moment of inertia
            dxdt[0] = 1 / actuator.motor.inductance * (-actuator.motor.resistance * x[0] 
                                                       -actuator.motor.BEMFConst * actuator.gear.ratio * x[1]
                                                       +actuator.motor.voltage);

            dxdt[1] = actuator.motor.torqueConst * x[0] / (actuator.gear.ratio * totalIM) -
                      actuator.spindle.radius * actuator.elasticForce /
                      (actuator.gear.ratio * actuator.gear.ratio * totalIM * actuator.gear.appEfficiency);
            
        }, x, time.toSec(), (period.toSec()/1000 /* devide by 1000 to obtain plausible results. Why needs further investigation*/) );

        applySpindleAngVel( x[1] );
        applyMotorCurrent( x[0], x[1] );

        // calculate resulting actuatorforce
        actuatorForce = 0.9 * actuator.ElectricMotorModel(actuator.motor.current, actuator.motor.torqueConst,
                                                    actuator.spindle.radius);

        //ROS_INFO_THROTTLE(1, "electric current: %.5f, speed: %.5f, force %.5f", actuator.motor.current,
        //                  actuator.spindle.angVel, actuatorForce);

        ros::spinOnce();

        // update gearposition
        actuator.gear.position += actuator.spindle.angVel * period.toSec();
        // update tendonLength
        tendonLength = initialTendonLength - actuator.spindle.radius * actuator.gear.position;

        publishTopics();

        //    ROS_INFO("electric current: %.5f, angVel: %.5f, actuator.force %.5f, see.force: %f", actuator.motor.current,
        //                   actuator.spindle.angVel, actuatorForce, see.see.force);
        //    ROS_INFO("tendonLength: %f, muscleLength: %f", tendonLength, muscleLength ); 
    
    }
    /////////////////////////////////////////////
    /// Setup topics for different motor values
    void IMuscle::setupTopics(){
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        char topic[100];
        snprintf(topic, 100, "/roboy/motor/actuatorForce");
        actuatorForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/seeForce");
        seeForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/motorCurrent");
        motorCurrent_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/spindleAngVel");
        spindleAngVel_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/muscleLength");
        muscleLength_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/tendonLength");
        tendonLength_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
    }
    //////////////////////////////////////////////
    // publishes motor information 
    void IMuscle::publishTopics(){
        std_msgs::Float32 msg;
        msg.data = actuatorForce;
        actuatorForce_pub.publish(msg);

        msg.data = see.see.force;
        seeForce_pub.publish(msg);
    
        msg.data = actuator.motor.current;
        motorCurrent_pub.publish(msg);

        msg.data = actuator.spindle.angVel;
        spindleAngVel_pub.publish(msg);
                
        msg.data = (muscleLength) * 100;
        muscleLength_pub.publish(msg);

        msg.data = tendonLength *100 ;
        tendonLength_pub.publish(msg);
    }
    ////////////////////////////////////////////////////
    // The Viapoint Wraping-type gets instantiated 
    // and built into a Linked list
    void IMuscle::initViaPoints( MyoMuscleInfo &myoMuscle ){
        for (int i = 0; i < myoMuscle.viaPoints.size(); i++) {
            ViaPointInfo vp = myoMuscle.viaPoints[i];
            if (vp.type == IViaPoints::FIXPOINT) {
                std::shared_ptr<IViaPoints> ptr(new IViaPoints(vp.point, vp.link));
                viaPoints.push_back(ptr);
            } else if (vp.type == IViaPoints::SPHERICAL) {
                std::shared_ptr<SphericalWrapping> ptr(
                        new SphericalWrapping(vp.point, vp.radius, vp.state, vp.revCounter, vp.link));
                viaPoints.push_back(ptr);
                ROS_INFO("state %d", vp.state);
            } else if (vp.type == IViaPoints::CYLINDRICAL) {
                ROS_INFO("state %d", vp.state);
                std::shared_ptr<CylindricalWrapping> ptr(
                        new CylindricalWrapping(vp.point, vp.radius, vp.state, vp.revCounter, vp.link));
                viaPoints.push_back(ptr);
                ROS_INFO("state %d", vp.state);
            } else if (vp.type == IViaPoints::MESH) {
                //TODO
            }
        }
        //linked list
        for (int i = 0; i < viaPoints.size(); i++) {
            if (i > 0) {
                viaPoints[i]->prevPoint = viaPoints[i - 1];
                viaPoints[i - 1]->nextPoint = viaPoints[i];
            }
        }
    }
    /////////////////////////////////////////////////
    // Calculates how the force goes along the tendon by going throught the Viapoint 
    void IMuscle::calculateTendonForceProgression(){
        for (int i = 0; i < viaPoints.size(); i++) {
            if (viaPoints[i]->prevPoint && viaPoints[i]->nextPoint) {
                viaPoints[i]->fa = viaPoints[i]->prevPoint->fb;
                viaPoints[i]->fb = viaPoints[i]->prevPoint->fb;
            } else if (!viaPoints[i]->prevPoint) {
                viaPoints[i]->fa = 0;
                viaPoints[i]->fb = muscleForce;
            } else if (!viaPoints[i]->nextPoint) {
                viaPoints[i]->fa = viaPoints[i]->prevPoint->fb;
                viaPoints[i]->fb = 0;
            }
            //CalculateForce differs for each wraping-type
            viaPoints[i]->CalculateForce();
        }
    }
    //////////////////////////////////////////////////////
    // adds physical restrictions to motor current
    void IMuscle::applyMotorCurrent( double &motorCurrent, const double &spindleAngVel ){
        //Motor has a max current it can handle while spinning
        if( spindleAngVel > 0 && motorCurrent > actuator.motor.continuousCurrent ){
            actuator.motor.current = motorCurrent = actuator.motor.continuousCurrent;
        }else{
                actuator.motor.current = motorCurrent;
        }
    }
    //////////////////////////////////////////////////////
    // adds physical restrictions to motor anglVel 
	void IMuscle::applySpindleAngVel( double &spindleAngVel ){
        //der mottor kann sich nur solange drehen wie sich die kraft gegen die dreht unter der stallforce liegt. Da der moter über eine feder zeiht 
        //kann er die doppelte kraft über den so entstehenden seilzugmechanismus an der feder aufbringen.   
            if( spindleAngVel > 0 && ( see.deltaX >= 0.02 || actuator.elasticForce >= actuator.motor.continuousTorque * actuator.gear.ratio / actuator.spindle.radius ) ){
                actuator.spindle.angVel = spindleAngVel = 0;
            //}else if( x[1] < 0 && see.see.force <= actuator.motor.stallTorque*actuator.gear.ratio / actuator.spindle.radius ){
            //    actuator.spindle.angVel = spindleAngVel = 0;
            }else{
                actuator.spindle.angVel = spindleAngVel;
            }
        }
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::IMuscle, roboy_simulation::IMuscle)
