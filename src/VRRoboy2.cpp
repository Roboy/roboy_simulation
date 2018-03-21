#include "roboy_simulation/VRRoboy2.hpp"

VRRoboy2::VRRoboy2(){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "VRRoboy2", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/pose", 100);
    muscle_state_pub = nh->advertise<roboy_communication_middleware::MuscleState>("/roboy/muscle_state", 100);
    external_force_sub = nh->subscribe("/roboy/external_force", 1, &VRRoboy2::applyExternalForce, this);
}

VRRoboy2::~VRRoboy2(){

}


void VRRoboy2::initializeWorlds(uint numberOfWorlds){
    printf("initializing worlds-...");
    // load numberOfWorlds empty worlds
    for (uint i = 0; i < numberOfWorlds; i++) {
        world.push_back(loadWorld("worlds/empty.world"));
    }
    printf("Worlds initialized.\n");
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();
    printf("Adding models...");
    // load the legs in each world
    for (uint i = 0; i < numberOfWorlds; i++) {
        physics::ModelPtr m = loadModel(world[i], "Roboy_simplified_moveable" );
        //RoboyModel = m;
        printf("initialized roboy for world nr %i. Adding hand models....\n", i);
        //physics::ModelPtr hands = loadModel(world[i], "hands");
        printf("Number of plugins: %i \n" , m->GetPluginCount());
        //FIXME The plugins are not loaded -> this simulation is useless for my purposes
        m->LoadPlugins();

        if (m != nullptr) {
            m->SetGravityMode(false); // quote: False to turn gravity on for the model
            //m->SetSelfCollide(false);
            model.push_back(m);
        }
    }
    printf("Models added.\n");
}

void VRRoboy2::publishPose(uint modelNr){
    roboy_communication_middleware::Pose msg;
    for(auto link:model[modelNr]->GetLinks()){
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

void VRRoboy2::publishMotorStates(uint modelNr){
    roboy_communication_middleware::MuscleState msg;
    for(auto link:model[modelNr]->GetLinks()){
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
        msg.motor_status.pwmRef.push_back((rand()/RAND_MAX));
        msg.motor_status.position.push_back((rand()/RAND_MAX));
        msg.motor_status.velocity.push_back((rand()/RAND_MAX));
        msg.motor_status.displacement.push_back((rand()/RAND_MAX));
        msg.motor_status.current.push_back((rand()/RAND_MAX));
    }
    muscle_state_pub.publish(msg);
}

void VRRoboy2::applyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg) {
    for(uint i=0; i<model.size(); i++){
        physics::LinkPtr link = model[i]->GetChildLink(msg->name);
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
    VRRoboy2.initializeWorlds(1);
    while(ros::ok()){
        // TODO this is from the previous coders... extremely dirty -> should all worlds be simulated or not?
        // Since I'm only using one for now, I don't care too much
        VRRoboy2.simulate(VRRoboy2.world[0]);
        VRRoboy2.publishPose(0);
        //VRRoboy2.publishMesh(RoboyModel)
        //VRRoboy2.publishMotorStates(0);
    }
}
