#include "roboy_simulation/VRRoboy2.hpp"

VRRoboy2::VRRoboy2(){
    //setup ros connection
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "VRRoboy2", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    //load world
    world = loadWorld("worlds/empty.world");
    printf("Worlds initialized.\n");

    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    // load model in world
    model = loadModel(world, "Roboy_simplified_moveable" );
    printf("Model added\n");
    if (model != nullptr) {
        //FIXME The plugins are not loaded -> this simulation does not load SimulateRoboyInRViz plugin
        //UPDATE: somehow. some were loaded!?? model->LoadPlugins();
        model->SetGravityMode(false); // quote user manual: "False to turn gravity on for the model"
        model->SetSelfCollide(false);
        printf("Number of plugins loaded: %i \n" , model->GetPluginCount());
    }

    //setup publisher and subscriber
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/pose", 100);
    muscle_state_pub = nh->advertise<roboy_communication_middleware::MuscleState>("/roboy/muscle_state", 100);
    external_force_sub = nh->subscribe("/roboy/external_force", 1, &VRRoboy2::applyExternalForce, this);
}

VRRoboy2::~VRRoboy2(){

}

void VRRoboy2::publishPose(){
    roboy_communication_middleware::Pose msg;
    for(auto link:model->GetLinks()){
        msg.name.push_back(link->GetName());
        math::Pose p = link->GetWorldPose();
        msg.x.push_back(p.pos.x);
        msg.y.push_back(p.pos.y);
        msg.z.push_back(p.pos.z);
        p.rot.Normalize();
        msg.qx.push_back(p.rot.x);
        msg.qy.push_back(p.rot.y);
        msg.qz.push_back(p.rot.z);
        msg.qw.push_back(p.rot.w);
    }
    pose_pub.publish(msg);
}

void VRRoboy2::applyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg) {
    physics::LinkPtr link = model->GetChildLink(msg->name);
    if(link != nullptr){
        math::Vector3 force(msg->f_x, msg->f_y, msg->f_z);
        math::Vector3 world_pos(msg->x, msg->y, msg->z);
        link->AddForceAtWorldPosition(force, world_pos);
    }
}

int main(int _argc, char **_argv){
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    VRRoboy2 VRRoboy2;
    //simulate and publish
    while(ros::ok()){
        VRRoboy2.simulate(VRRoboy2.world);
        VRRoboy2.publishPose();
    }
}
