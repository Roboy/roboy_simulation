#include "roboy_simulation/ForceJointPlugin.hpp"

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ForceJointPlugin)

ForceJointPlugin::ForceJointPlugin() : ModelPlugin() {}

ForceJointPlugin::~ForceJointPlugin(){}

void ForceJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // get the model
    model = _parent;
    // bind the gazebo update function to OnUpdate
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ForceJointPlugin::OnUpdate, this, _1));
    // get all joints and the initial pose
    physics::Joint_V jointVector = model->GetJoints();
    initPose = model->GetWorldPose();

    // Init ros if it is has not been initialized
    if(!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "PaBiRoboy");
    }

    // Create ros node
    nh = ros::NodeHandlePtr(new ros::NodeHandle("roboy"));
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    jointCommandRevolute_sub = nh->subscribe("/roboy/middleware/JointCommandRevolute", 1, &ForceJointPlugin::JointCommandRevolute, this);
    jointCommandRevolute2_sub = nh->subscribe("/roboy/middleware/JointCommandRevolute2", 1, &ForceJointPlugin::JointCommandRevolute2, this);
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/simulation/"+ _parent->GetName() +"_pose", 1);
    hip_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensor_location", 1, &ForceJointPlugin::DarkRoomSensor, this);
    for(auto joint = jointVector.begin(); joint != jointVector.end(); joint++)
    {   
        
         // Test if joint type is revolute2
        if((*joint)->GetType() == 320){
            cout << (*joint)->GetName()<< endl;
            // replace whitespace with underscore in the names
            string _modelName = model->GetName();
            string jointName = (*joint)->GetName();
            string _jointName = jointName;
            boost::algorithm::replace_all(_modelName, " ", "_");
            boost::algorithm::replace_all(_jointName, " ", "_");
            jointsRevolute2.push_back(jointName);
            jointAnglesRevolute2[jointName][0] = (*joint)->GetAngle(0).Radian();
            jointAnglesRevolute2[jointName][1]= (*joint)->GetAngle(1).Radian();

        }
        // Test if joint type is revolute
        else if((*joint)->GetType() == 576){
            // replace whitespace with underscore in the names
            string _modelName = model->GetName();
            string jointName = (*joint)->GetName();
            string _jointName = jointName;
            boost::algorithm::replace_all(_modelName, " ", "_");
            boost::algorithm::replace_all(_jointName, " ", "_");
            jointsRevolute.push_back(jointName);
            jointAnglesRevolute[jointName] = (*joint)->GetAngle(0).Radian();
        }
    }
}

void ForceJointPlugin::publishPose()
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

void ForceJointPlugin::JointCommandRevolute(const roboy_communication_middleware::JointCommandRevoluteConstPtr &msg){
        for(uint i=0;i<msg->link_name.size();i++){
        jointAnglesRevolute[msg->link_name[i]] = msg->angle[i];
    }

}
void ForceJointPlugin::JointCommandRevolute2(const roboy_communication_middleware::JointCommandRevolute2ConstPtr &msg){
        for(uint i=0;i<msg->link_name.size();i++){
        jointAnglesRevolute2[msg->link_name[i]][0] = msg->angle1[i];
        jointAnglesRevolute2[msg->link_name[i]][1] = msg->angle2[i];
    }

}



void ForceJointPlugin::DarkRoomSensor(const roboy_communication_middleware::DarkRoomSensorConstPtr &msg)
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

void ForceJointPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    // make the model stationary
    publishPose();
    model->SetWorldPose(initPose);
    // set velocity and force to zero and force for every saved joint and set angle to saved value
    for(auto it = jointsRevolute.begin(); it != jointsRevolute.end(); it++)
    {
        model->GetJoint(*it)->SetVelocity(0, 0);
        model->GetJoint(*it)->SetForce(0, 0);
        model->GetJoint(*it)->SetPosition(0, jointAnglesRevolute[*it]);
    }

    for(auto it = jointsRevolute2.begin(); it != jointsRevolute2.end(); it++)
    {
        model->GetJoint(*it)->SetVelocity(0, 0);
        model->GetJoint(*it)->SetForce(0, 0);
        //TODO: Set Position of axis one and two
    }
    
}


