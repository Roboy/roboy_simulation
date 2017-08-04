#include "roboy_simulation/BeRoboyPlugin.hpp"
#include <math.h>

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BeRoboyPlugin)

BeRoboyPlugin::BeRoboyPlugin() : ModelPlugin() {}

BeRoboyPlugin::~BeRoboyPlugin(){}

void BeRoboyPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // get the model
    model = _parent;
    // bind the gazebo update function to OnUpdate
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BeRoboyPlugin::OnUpdate, this, _1));
    // get all joints and the initial pose
    physics::Joint_V jointVector = model->GetJoints();
    initPose = model->GetWorldPose();

    // Init ros if it is has not been initialized
    if(!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "BeRoboy");
    }

    // Create ros node
    nh = ros::NodeHandlePtr(new ros::NodeHandle("BeRoboy"));
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    jointCommand_sub = nh->subscribe("/roboy/middleware/JointCommand", 1, &BeRoboyPlugin::JointCommand, this);
    setPosition_sub = nh->subscribe("/roboy/middleware/Position", 1, &BeRoboyPlugin::SetPosition, this);
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/simulation/"+ _parent->GetName() +"_pose", 1);
    hip_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensor_location", 1, &BeRoboyPlugin::DarkRoomSensor, this);
    for(auto joint = jointVector.begin(); joint != jointVector.end(); joint++)
    {
        // Test if joint type is revolute
        if((*joint)->GetType() != 576)
            continue;
        // replace whitespace with underscore in the names
        string _modelName = model->GetName();
        string jointName = (*joint)->GetName();
        string _jointName = jointName;
        boost::algorithm::replace_all(_modelName, " ", "_");
        boost::algorithm::replace_all(_jointName, " ", "_");
        joints.push_back(jointName);
        jointAngles[jointName] = (*joint)->GetAngle(0).Radian();
    }
}

void BeRoboyPlugin::publishPose()
{
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

void BeRoboyPlugin::JointCommand(const roboy_communication_middleware::JointCommandConstPtr &msg){
    for(uint i=0;i<msg->link_name.size();i++){
        jointAngles[msg->link_name[i]] = msg->angle[i];
    }
}

void BeRoboyPlugin::SetPosition(const roboy_communication_middleware::PositionConstPtr &msg){
	math::Vector3 pos(msg->x, msg->y, msg->z);
    	gazebo::math::Pose p(pos, initPose.rot);
	initPose = p;
}

void BeRoboyPlugin::DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensorConstPtr &msg)
{
    int hipIDPos = -1;
    for(int i = 0; i < msg->ids.size(); i++)
    {
	// hip id of the sensor should be 4
	if(msg->ids[i] == hipID)
	{
	    hipIDPos = msg->ids[i];
	    break;
	}
    }
    if(hipIDPos == -1)
	return;
    
    // move the position of the model
    math::Quaternion modelRot = model->GetWorldPose().rot;
    math::Vector3 modelPos = math::Vector3(msg->position[hipIDPos].x, msg->position[hipIDPos].y, msg->position[hipIDPos].z);
    initPose = math::Pose(math::Pose(modelPos, modelRot));
}

void BeRoboyPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    // make the model stationary
    model->SetWorldPose(initPose);
    // set velocity and force to zero and force for every saved joint and set angle to saved value
    for(auto it = joints.begin(); it != joints.end(); it++)
    {
        model->GetJoint(*it)->SetVelocity(0, 0);
        model->GetJoint(*it)->SetForce(0, 0);
        model->GetJoint(*it)->SetPosition(0, jointAngles[*it]);
    }
	initPose = model->GetWorldPose();
	publishPose();
    
}
