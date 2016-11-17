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
        msg.roll.push_back(p.rot.x);
        msg.pitch.push_back(p.rot.y);
        msg.yaw.push_back(p.rot.z);
    }
    pose_pub.publish(msg);
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
    }
}