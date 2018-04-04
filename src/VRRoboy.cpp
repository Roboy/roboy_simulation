#include "roboy_simulation/VRRoboy.hpp"

VRRoboy::VRRoboy(){
    //ros / simulation setup
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "VRRoboy", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    // load empty world
    world = loadWorld("worlds/empty.world");

    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    // load model into world
    model = loadModel(world, "legs_with_upper_body_simplified" );
    physics::ModelPtr hands = loadModel(world, "hands");

    //advertise positions
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/pose", 100);
    //advertise muscles
    muscle_state_pub = nh->advertise<roboy_communication_middleware::MuscleState>("/roboy/muscle_state", 100);
    //subscribe to external forces
    external_force_sub = nh->subscribe("/roboy/external_force", 1, &VRRoboy::applyExternalForce, this);}

VRRoboy::~VRRoboy(){
    free(&model);
    free(&world);
}

void VRRoboy::publishPose(){
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


void VRRoboy::publishRandomMotorStates(){
    roboy_communication_middleware::MuscleState msg;
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
        msg.motor_status.pwmRef.push_back((rand()/RAND_MAX));
        msg.motor_status.position.push_back((rand()/RAND_MAX));
        msg.motor_status.velocity.push_back((rand()/RAND_MAX));
        msg.motor_status.displacement.push_back((rand()/RAND_MAX));
        msg.motor_status.current.push_back((rand()/RAND_MAX));
    }
    muscle_state_pub.publish(msg);
}


void VRRoboy::applyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg) {
    //try find link with specified name
    physics::LinkPtr link = model->GetChildLink(msg->name);
    if(link) {
        // Apply force with relative position on given link
        math::Vector3 force(msg->f_x, msg->f_y, msg->f_z);
        math::Vector3 relative_pos(msg->x, msg->y, msg->z);
        link->AddForceAtRelativePosition(force, relative_pos);
    }
}

int main(int _argc, char **_argv){
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed! Aborting simulation.\n";
        return 1;
    }

    VRRoboy vrRoboy;

    //run simulation & publish
    while(ros::ok()){
        vrRoboy.simulate(vrRoboy.world);
        vrRoboy.publishPose();
        vrRoboy.publishRandomMotorStates();
    }
    return 0;
}

