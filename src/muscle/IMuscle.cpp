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
        initViaPoints(myoMuscle);

        actuator.motor = myoMuscle.motor;
        actuator.gear = myoMuscle.gear;
        actuator.spindle = myoMuscle.spindle;
        see.see = myoMuscle.see;
        name = myoMuscle.name;
        see.see.expansion = 0.0;
        see.see.force = 0.0;
    }

    void IMuscle::Update(ros::Time &time, ros::Duration &period) {
        //ROS_INFO("TIME:  %f", time.toSec());
        /*if(time.toSec() >= 1){
            pid_control = true;
            cmd = 0.01;
            feedback_type = 2;
        }
        if(time.toSec() >= 10){
            cmd = 0.005;
        }*/

        if (pid_control) {
            actuator.motor.voltage = musclePID.calculate(period.toSec(), cmd, feedback[feedback_type]);
        } else {
            actuator.motor.voltage = cmd * 24;//simulated PWM
        }

        for (int i = 0; i < viaPoints.size(); i++) {
            viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
                                              viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);

            // absolute position + relative position=actual position of each via point
//            switch (viaPoints[i]->type){
//                case IViaPoints::FIXPOINT:
//                    viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
//                                                      viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);
//                    break;
//                case IViaPoints::CYLINDRICAL:
//                    if(this->spanningJoint!=nullptr)
//                        viaPoints[i]->globalCoordinates = this->spanningJoint->GetWorldPose().pos;
//                    break;
//                case IViaPoints::SPHERICAL:
//                    if(this->spanningJoint!=nullptr)
//                        viaPoints[i]->globalCoordinates = this->spanningJoint->GetWorldPose().pos;
//                    break;
//                case IViaPoints::MESH:
//                    viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
//                                                      viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);
//                    break;
//            }
        }

        //update force points and calculate muscle length for each Viapoint
        //muscleLength is set zero and then added up again
        if (!firstUpdate) {
            prevMuscleLength = muscleLength;
        }
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
            prevMuscleLength = muscleLength;
            initialTendonLength = tendonLength = muscleLength + see.internalLength;
            ROS_INFO("inital tendon length is %f m", initialTendonLength);
        }

        //calculate elastic force
        see.ElasticElementModel(tendonLength, muscleLength);
        see.applyTendonForce(muscleForce, actuator.elasticForce);


        // calculate the approximation of gear's efficiency
        actuator.gear.appEfficiency = actuator.EfficiencyApproximation();

        // do 1 step of integration of DiffModel() at current time
        boost::numeric::odeint::integrate([this](const IActuator::state_type &x, IActuator::state_type &dxdt, double t) {
            // This lambda function describes the differential model for the simulations of dynamics
            // of a DC motor, a spindle, and a gear box`
            // x[0] - motor electric current
            // x[1] - spindle angular velocity
            double totalIM = actuator.motor.inertiaMoment + actuator.gear.inertiaMoment; // total moment of inertia
            dxdt[0] = 1.0 / actuator.motor.inductance * (-actuator.motor.resistance * x[0]
                                                         - actuator.motor.BEMFConst * actuator.gear.ratio * x[1]
                                                         + actuator.motor.voltage);
            dxdt[1] = actuator.motor.torqueConst * x[0] / (actuator.gear.ratio * totalIM) -
                      actuator.spindle.radius * actuator.elasticForce /
                      (actuator.gear.ratio * actuator.gear.ratio * totalIM * actuator.gear.appEfficiency);
        }, x, time.toSec(), time.toSec()+period.toSec(), period.toSec());

        actuator.motor.current = x[0];
        actuator.spindle.angVel = x[1];

        // update gearposition
        actuator.gear.position += actuator.spindle.angVel * period.toSec();
        // update tendonLength
        tendonLength = initialTendonLength - 2 * 3.141 * actuator.spindle.radius * actuator.gear.position;

        //calculate elastic force again after actuation. without the second update the motor will be a step ahead of the simulation. the spring is the comunication of force between robot and motor.
        see.ElasticElementModel(tendonLength, muscleLength);
        see.applyTendonForce(muscleForce, actuator.elasticForce);

        calculateTendonForceProgression();

        ros::spinOnce();

        // feedback for PID-controller
        feedback[0] = muscleForce;
        feedback[1] = actuator.gear.position;
        feedback[2] = see.deltaX;

        publishTopics();

        if(firstUpdate)
            firstUpdate = false;

        //    ROS_INFO_THROTTLE(1, "electric current: %.5f, speed: %.5f, force %.5f", actuator.motor.current,
        //                  actuator.spindle.angVel, muscleForce);
        //    ROS_INFO("electric current: %.5f, angVel: %.5f, muscleForce %.5f, springDis: %f", actuator.motor.current,
        //                   actuator.spindle.angVel, muscleForce, see.deltaX);
        //    ROS_INFO("tendonLength: %f, muscleLength: %f", tendonLength, muscleLength ); 

    }

    /////////////////////////////////////////////
    /// Setup topics for different motor values
    void IMuscle::setupTopics() {
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        char topic[100];
        snprintf(topic, 100, "/roboy/motor/muscleForce");
        muscleForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/seeForce");
        seeForce_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/motorCurrent");
        motorCurrent_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/spindleAngVel");
        spindleAngVel_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/totalLength");
        totalLength_pub = nh->advertise<std_msgs::Float32>(topic, 1000);
        snprintf(topic, 100, "/roboy/motor/tendonLength");
        tendonLength_pub = nh->advertise<std_msgs::Float32>(topic, 1000);

    }

    //////////////////////////////////////////////
    // publishes motor information 
    void IMuscle::publishTopics() {
        std_msgs::Float32 msg;

        msg.data = muscleForce;
        muscleForce_pub.publish(msg);

        // publisch spring displacemment or spring force:
        msg.data = see.deltaX; //(see.deltaX >= 0) ? (see.deltaX * 30680.0) : 0;
        seeForce_pub.publish(msg);

        msg.data = actuator.motor.current;
        motorCurrent_pub.publish(msg);

        msg.data = actuator.spindle.angVel;
        spindleAngVel_pub.publish(msg);

        msg.data = tendonLength * 1;
        totalLength_pub.publish(msg);

        msg.data = (muscleLength + see.internalLength) * 1;
        tendonLength_pub.publish(msg);
    }

    ////////////////////////////////////////////////////
    // The Viapoint Wraping-type gets instantiated 
    // and built into a Linked list
    void IMuscle::initViaPoints(MyoMuscleInfo &myoMuscle) {
        static int message_counter = 6666;
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
    void IMuscle::calculateTendonForceProgression() {
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
}
// make it a plugin loadable via pluginlib
PLUGINLIB_EXPORT_CLASS(roboy_simulation::IMuscle, roboy_simulation::IMuscle)
