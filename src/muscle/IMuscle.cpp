#include "roboy_simulation/muscle/IMuscle.hpp"


namespace roboy_simulation {

    IMuscle::IMuscle() : muscleLength(0), tendonLength(0), initialTendonLength(0), firstUpdate(true) {
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "MusclePlugin",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        x.resize(2);
        char topic[100];
        //snprintf(topic, 100, "/roboy%d/%s/actuatorForce", roboyID, name.c_str());
        snprintf(topic, 100, "/roboy/test/actuatorForce");
        actuatorForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/test/seeForce");
        seeForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/test/motorCurrent");
        motorCurrent_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/test/spindleAngVel");
        spindleAngVel_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/test/muscleLength");
        muscleLength_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/test/tendonLength");
        tendonLength_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
    }

    void IMuscle::Init(MyoMuscleInfo &myoMuscle) {
        //state initialization
        x[0] = 0.0;
        x[1] = 0.0;
        actuator.motor.voltage = 0.0;
        actuator.spindle.angVel = 0;
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

            }
        }

        //linked list
        for (int i = 0; i < viaPoints.size(); i++) {
            if (i > 0) {
                viaPoints[i]->prevPoint = viaPoints[i - 1];
                viaPoints[i - 1]->nextPoint = viaPoints[i];
            }
        }

        actuator.motor = myoMuscle.motor;
        actuator.gear = myoMuscle.gear;
        actuator.spindle = myoMuscle.spindle;
        see.see = myoMuscle.see;
        name = myoMuscle.name;
        see.see.expansion = 0.0;
        see.see.force = 0.0;

        muscle_type = myoMuscle.muscle_type;
        spanningJoint = myoMuscle.spanningJoint;
    }

    void IMuscle::Update(ros::Time &time, ros::Duration &period) {
        actuator.motor.voltage = cmd;

        for (int i = 0; i < viaPoints.size(); i++) {
            // absolute position + relative position=actual position of each via point
            viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
                                              viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);
        }

        //update force points and calculate muscle length
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
            ROS_INFO("Calculated musclelength is %f m", muscleLength);
            tendonLength = initialTendonLength = muscleLength + see.internalLength;
            firstUpdate = false;
        }

        //calculate elastic force
        //see.see.length = muscleLength - tendonLength;
        //see.ElasticElementModel(see.see, see.see.length);
        see.ElasticElementModel( tendonLength, muscleLength, initialTendonLength );
        actuator.elasticForce = see.see.force;
        //set elastic force zero to compare with old plugin functionality
        //actuator.elasticForce = 0;
        for (int i = 0; i < viaPoints.size(); i++) {
#ifdef DUMMYMUSCLE
            //ROS_INFO("DUMMYMUSCLE is defined");
            viaPoints[i]->fa = cmd;
            viaPoints[i]->fb = cmd;
#else
            
            if (viaPoints[i]->prevPoint && viaPoints[i]->nextPoint) {
                viaPoints[i]->fa = viaPoints[i]->prevPoint->fb;
//____________________________________________________________________________________________________________________________________
                viaPoints[i]->fb = viaPoints[i]->prevPoint->fb; // viaPoints[i]->fa; 
//____________________________________________________________________________________________________________________________________
               
            } else if (!viaPoints[i]->prevPoint) {
                viaPoints[i]->fa = 0;
                //use this to compare with old functionality of plugin
                viaPoints[i]->fb = actuator.elasticForce + actuatorForce;
                //viaPoints[i]->fb = see.see.force + actuatorForce;
            } else if (!viaPoints[i]->nextPoint) {
                viaPoints[i]->fa = viaPoints[i]->prevPoint->fb;
                viaPoints[i]->fb = 0;
            }
#endif
            viaPoints[i]->CalculateForce();
            if(i>0) {
                if (viaPoints[i-1]->link != viaPoints[i]->link) {
                    math::Vector3 v = viaPoints[i]->globalCoordinates - viaPoints[i-1]->globalCoordinates;
                    math::Vector3 w = spanningJoint->GetWorldPose().pos - viaPoints[i-1]->globalCoordinates;
                    momentArm = v.Normalize()*w.Dot(v.Normalize()) - w;
                }
            }

        };
//____________________________________________________________________________________________________________________________________
        // calculate the approximation of gear's efficiency

        actuator.gear.appEfficiency = actuator.EfficiencyApproximation();
//____________________________________________________________________________________________________________________________________
  //Test output:
        // double test = 1 / actuator.motor.inductance * (-actuator.motor.resistance * x[0] - actuator.motor.BEMFConst * actuator.gear.ratio * x[1] + actuator.motor.voltage);
        // ROS_INFO("%f V -> dxdt[0] = %f", actuator.motor.voltage, test);
        // ROS_INFO( "1 / %f * (-%f * %f - %f * %f * %f + %f)", actuator.motor.inductance, actuator.motor.resistance,x[0],  actuator.motor.BEMFConst, actuator.gear.ratio,x[1],  actuator.motor.voltage);
        // ROS_INFO( "x[0] = %f;   x[1] = %f",x[0], x[1]);
//____________________________________________________________________________________________________________________________________
        ROS_INFO("=====================================");
        ROS_INFO("Voltage: %f", actuator.motor.voltage);
        //ROS_INFO("________Pre-Stepper__________________");
        //ROS_INFO("x[0]: %f;     x[1]: %f", x[0], x[1]);
//____________________________________________________________________________________________________________________________________
        // do 1 step of integration of DiffModel() at current time        
        actuator.stepper.do_step([this](const IActuator::state_type &x, IActuator::state_type &dxdt, const double ) {
            // This lambda function describes the differential model for the simulations of dynamics
            // of a DC motor, a spindle, and a gear box`
            // x[0] - motor electric current
            // x[1] - spindle angular velocity
            double totalIM = actuator.motor.inertiaMoment + actuator.gear.inertiaMoment; // total moment of inertia
            dxdt[0] = 1 / actuator.motor.inductance * (-actuator.motor.resistance * x[0] -
                                                       actuator.motor.BEMFConst * actuator.gear.ratio * x[1] +
                                                       actuator.motor.voltage);

            dxdt[1] = actuator.motor.torqueConst * x[0] / (actuator.gear.ratio * totalIM) -
                      actuator.spindle.radius * actuator.elasticForce /
                      (actuator.gear.ratio * actuator.gear.ratio * totalIM * actuator.gear.appEfficiency);
            //ROS_INFO("dxdt[0]: %f;     dxdt[1]: %f", dxdt[0], dxdt[1]);
        }, x, time.toSec(), (period.toSec()/1000) );  

/*        
        sinParm += 10/180 * 3.141;      
        // Test integration Input -> sinus    
        actuator.stepper.do_step([this](const IActuator::state_type &x, IActuator::state_type &dxdt, const double) {
            dxdt[0] = sin( sinParm );
            dxdt[1] = sin( sinParm );
        }, x, time.toSec(), period.toSec());
*/
        
        actuator.motor.current = x[0];
        if( see.see.force >= actuator.motor.stallTorque*actuator.gear.ratio ){
            actuator.spindle.angVel = x[1] = 0;
        }else{actuator.spindle.angVel = x[1];}
        
        //calculate motor force
        actuatorForce = actuator.ElectricMotorModel(actuator.motor.current, actuator.motor.torqueConst,
                                                    actuator.spindle.radius);

        //ROS_INFO_THROTTLE(1, "electric current: %.5f, speed: %.5f, force %.5f", actuator.motor.current,
        //                  actuator.spindle.angVel, actuatorForce);

        ros::spinOnce();

        actuator.gear.position += actuator.spindle.angVel * period.toSec();
        // tendon length can go negative if motor keeps turnings
        tendonLength = initialTendonLength - actuator.spindle.radius * actuator.gear.position;

            std_msgs::Float32 msg;
            msg.data = actuatorForce;
            actuatorForce_pub.publish(msg);

            msg.data = see.see.force;
            seeForce_pub.publish(msg);
        
            msg.data = actuator.motor.current;
            motorCurrent_pub.publish(msg);

            msg.data = actuator.spindle.angVel;
            spindleAngVel_pub.publish(msg);
                    
            msg.data = muscleLength;
            muscleLength_pub.publish(msg);

            msg.data = tendonLength;
            tendonLength_pub.publish(msg);
        //    ROS_INFO("________Post-Stepper__________________");
        // ROS_INFO("x[0]: %f;     x[1]: %f", x[0], x[1]);
        // ROS_INFO("Time: %f;     Period:%f", time.toSec(), period.toSec());
        // ROS_INFO("______________________________");
        
            ROS_INFO("electric current: %.5f, angVel: %.5f, actuator.force %.5f, see.force: %f", actuator.motor.current,
                            actuator.spindle.angVel, actuatorForce, see.see.force);
            ROS_INFO("tendonLength: %f ", tendonLength ); 
    
    }
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::IMuscle, roboy_simulation::IMuscle)
