#include "roboy_simulation/ForceTorquePluginHalterung.hpp"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ForceTorquePluginHalterung)

ForceTorquePluginHalterung::ForceTorquePluginHalterung() : SensorPlugin() {
    // start ros node
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "force_torque_sensor_halterung",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;
}

ForceTorquePluginHalterung::~ForceTorquePluginHalterung() {
    delete nh;
}

void ForceTorquePluginHalterung::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get the parent sensor.
#if GAZEBO_MAJOR_VERSION < 7
    parentSensor = boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensor);
#else
    this->parentSensor = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensor);
#endif

    // Make sure the parent sensor is valid.
    if (!parentSensor) {
        gzerr << "ForceTorquePluginHalterung requires a ForceTorqueSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    updateConnection = parentSensor->ConnectUpdated(std::bind(&ForceTorquePluginHalterung::OnUpdate, this));

    // Make sure the parent sensor is active.
    parentSensor->SetActive(true);

    force_torque_pub = nh->advertise<roboy_simulation::ForceTorque>("/roboy/"+sdf->GetAttribute("name")->GetAsString(),1);
    roboyID_sub = nh->subscribe("/roboy/id", 1, &ForceTorquePluginHalterung::updateID, this);

    ROS_INFO_NAMED("force_torque_sensor","%s loaded", sdf->GetAttribute("name")->GetAsString().c_str());
}

void ForceTorquePluginHalterung::OnUpdate() {
    // Get all the contacts.
    roboy_simulation::ForceTorque msg;
    msg.roboyID = roboyID;
#if GAZEBO_MAJOR_VERSION < 7
    math::Vector3 force = parentSensor->GetForce();
    math::Vector3 torque = parentSensor->GetTorque();
    msg.force.x = force.x;
    msg.force.y = force.y;
    msg.force.z = force.z;
    msg.torque.x = torque.x;
    msg.torque.y = torque.y;
    msg.torque.z = torque.z;
#else
    ignition::math::Vector3d force = parentSensor->Force();
    ignition::math::Vector3d torque = parentSensor->Torque();
    msg.joint = parentSensor->Joint()->GetName();
    msg.force.x = force.X();
    msg.force.y = force.Y();
    msg.force.z = force.Z();
    msg.torque.x = torque.X();
    msg.torque.y = torque.Y();
    msg.torque.z = torque.Z();
#endif
    force_torque_pub.publish(msg);
    ros::spinOnce();
}

void ForceTorquePluginHalterung::updateID(const std_msgs::Int32::ConstPtr &msg){
    roboyID = msg->data;
}
