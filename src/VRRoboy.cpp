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
    pose_sub = nh->subscribe("/roboy/pose_test", 1000, &VRRoboy::publishTestPose, this);
    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);
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
        //physics::ModelPtr m = loadModel(world[i], "model://legs_with_upper_body" );
        physics::ModelPtr m = loadModel(world[i], "model://PaBiRoboy" );
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
        p.rot.Normalize();
        msg.qx.push_back(p.rot.x);
        msg.qy.push_back(p.rot.y);
        msg.qz.push_back(p.rot.z);
        msg.qw.push_back(p.rot.w);
    }
    pose_pub.publish(msg);
}

void VRRoboy::publishTestPose( const geometry_msgs::Pose::ConstPtr& msg ){
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "world";
    mesh.ns = "model_test";
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration(0);
    mesh.header.stamp = ros::Time::now();
    mesh.action = visualization_msgs::Marker::ADD;

    mesh.id = 0;
    mesh.pose.position.x = msg->position.x;
    mesh.pose.position.y = msg->position.y;
    mesh.pose.position.z = msg->position.z;
    mesh.pose.orientation.x = msg->orientation.x;
    mesh.pose.orientation.y = msg->orientation.y;
    mesh.pose.orientation.z = msg->orientation.z;
    mesh.pose.orientation.w = msg->orientation.w;

    mesh.mesh_resource = "package://roboy_models/legs_with_upper_body/cad/torso.STL";
    marker_visualization_pub.publish(mesh);
}

void VRRoboy::publishMotorStates(uint modelNr){
    common_utilities::MuscleState msg;
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