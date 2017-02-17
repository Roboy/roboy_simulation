#include "roboy_simulation/VRRoboy.hpp"

VRRoboy::VRRoboy(){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "VRRoboy", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    pose_pub = nh->advertise<common_utilities::Pose>("/roboy/pose", 100);
    muscle_state_pub = nh->advertise<common_utilities::MuscleState>("/roboy/muscle_state", 100);
    external_force_sub = nh->subscribe("/roboy/external_force", 1, &VRRoboy::applyExternalForce, this);
}

VRRoboy::~VRRoboy(){

}

void VRRoboy::initializeWorlds(uint numberOfWorlds){
    // load numberOfWorlds empty worlds
    for (uint i = 0; i < numberOfWorlds; i++) {
        world.push_back(loadWorld("worlds/empty.world"));
    }
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    // load the legs in each world
    for (uint i = 0; i < numberOfWorlds; i++) {
        physics::ModelPtr m = loadModel(world[i], "model://legs_with_upper_body" );
        if (m != nullptr) {
            model.push_back(m);
        }
    }
}

void VRRoboy::publishPose(uint modelNr){
    common_utilities::Pose msg;
    for(auto link:model[modelNr]->GetLinks()){
        msg.name.push_back(link->GetName());
        math::Pose p = link->GetWorldPose();
        msg.x.push_back(p.pos.x);
        msg.y.push_back(p.pos.y);
        msg.z.push_back(p.pos.z);
        math::Vector3 rot = p.rot.GetAsEuler();
        msg.roll.push_back(rot.x);
        msg.pitch.push_back(rot.y);
        msg.yaw.push_back(rot.z);
    }
    pose_pub.publish(msg);
}

void VRRoboy::publishMotorStates(uint modelNr){
    common_utilities::MuscleState msg;
    for(auto link:model[modelNr]->GetLinks()){
        msg.name.push_back(link->GetName());
        math::Pose p = link->GetWorldPose();
        msg.x.push_back(p.pos.x);
        msg.y.push_back(p.pos.y);
        msg.z.push_back(p.pos.z);
        math::Vector3 rot = p.rot.GetAsEuler();
        msg.roll.push_back(rot.x);
        msg.pitch.push_back(rot.y);
        msg.yaw.push_back(rot.z);
        msg.motor_status.jointPos.push_back((rand()/RAND_MAX)*5.0);
        msg.motor_status.actuatorPos.push_back((rand()/RAND_MAX)*5.0);
        msg.motor_status.actuatorVel.push_back((rand()/RAND_MAX)*5.0);
        msg.motor_status.actuatorCurrent.push_back((rand()/RAND_MAX)*5.0);
        msg.motor_status.tendonDisplacement.push_back((rand()/RAND_MAX)*5.0);
    }
    muscle_state_pub.publish(msg);
}

void VRRoboy::applyExternalForce(const common_utilities::ExternalForce::ConstPtr &msg) {
    for(uint i=0; i<model.size(); i++){
        physics::LinkPtr link = model[i]->GetChildLink(msg->name);
        math::Vector3 force(msg->f_x, msg->f_y, msg->f_z);
        math::Vector3 relative_pos(msg->x, msg->y, msg->z);
        link->AddForceAtRelativePosition(force,relative_pos);
        duration_in_milliseconds = msg->duration;
        apply_external_force = true;
    }
}

int main(int _argc, char **_argv){
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    VRRoboy vrRoboy;
    vrRoboy.initializeWorlds(1);

    while(ros::ok()){
        vrRoboy.simulate(vrRoboy.world[0]);
        vrRoboy.publishPose(0);
        vrRoboy.publishMotorStates(0);
        if(vrRoboy.apply_external_force){
            common::Time start = vrRoboy.world[0]->GetSimTime();
            while(((vrRoboy.world[0]->GetSimTime()-start).nsec*common::Time::nsInMs) <= vrRoboy.duration_in_milliseconds){
                vrRoboy.simulate(vrRoboy.world[0]);
            }
        }
    }
}