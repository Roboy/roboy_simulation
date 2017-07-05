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

    jointCommand_sub = nh->subscribe("/roboy/middleware/JointCommand", 1, &ForceJointPlugin::JointCommand, this);
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/simulation/pabi_pose", 1);

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

void ForceJointPlugin::JointCommand(const roboy_communication_middleware::JointCommandConstPtr &msg){
    for(uint i=0;i<msg->link_name.size();i++){
        jointAngles[msg->link_name[i]] = msg->angle[i];
    }
}

void ForceJointPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    publishPose();
    // make the model stationary! NOW MOVED TO SDF FILE!
    // model->SetWorldPose(initPose);
    // set velocity and force to zero and force for every saved joint and set angle to saved value
    for(auto it = joints.begin(); it != joints.end(); it++)
    {
        model->GetJoint(*it)->SetVelocity(0, 0);
        model->GetJoint(*it)->SetForce(0, 0);
        model->GetJoint(*it)->SetPosition(0, jointAngles[*it]);
    }
}
