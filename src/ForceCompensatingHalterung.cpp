#include "ForceCompensatingHalterung.hpp"

ForceCompensatingHalterung::ForceCompensatingHalterung(){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ForceCompensatingHalterung",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    force_torque_halterung_sub = nh->subscribe("/roboy/force_torque_halterung", 1,
                                                &ForceCompensatingHalterung::updateForce, this);

    roboyID_sub = nh->subscribe("/roboy/id", 1, &ForceCompensatingHalterung::updateID, this);

    ros::spinOnce();
}

ForceCompensatingHalterung::~ForceCompensatingHalterung(){

}

void ForceCompensatingHalterung::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
    ROS_INFO("Loading ForceCompensatingHalterung plugin");
    // Save pointers to the model
    parent_model = parent_;
    sdf = sdf_;

    // Error message if the model couldn't be found
    if (!parent_model) {
        ROS_ERROR_STREAM_NAMED("loadThread", "parent model is NULL");
        return;
    }

    // Check that ROS has been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ForceCompensatingHalterung::Update, this));

    ROS_INFO("ForceCompensatingHalterung ready");
}

void ForceCompensatingHalterung::Update() {
    // Get the simulation time
    gz_time_now = parent_model->GetWorld()->GetSimTime();

    ros::spinOnce();
    // compensate the force acting on the halterung
    physics::LinkPtr halterung = parent_model->GetLink("halterung");
//    math::Vector3 acc = compensation_force;
    halterung->AddForce(compensation_force);
//    ROS_INFO_STREAM( acc);
}

void ForceCompensatingHalterung::updateID(const std_msgs::Int32::ConstPtr &msg){
    roboyID = msg->data;
}

void ForceCompensatingHalterung::updateForce(const roboy_simulation::ForceTorque::ConstPtr &msg){
    compensation_force.x = msg->force.x;
    compensation_force.y = msg->force.y;
    compensation_force.z = 0;
}

GZ_REGISTER_MODEL_PLUGIN(ForceCompensatingHalterung)