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
        ros::init(argc, argv, "gazebo_client" , ros::init_options::NoSigintHandler);
    }

    // Create ros node
    nh.reset(new ros::NodeHandle("roboy"));

    // Create a named topic, and subscribe to it for every joint
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
        string subPath = "pabi_angle/" + _jointName;
        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(subPath,
                                                                                    1,
                                                                                    boost::bind(&ForceJointPlugin::OnRosMsg, this, _1, jointName),
                                                                                    ros::VoidPtr(), &this->rosQueue);
        rosSubList.push_back(nh->subscribe(so));
        joints.push_back(jointName);
        jointAngles[jointName] = (*joint)->GetAngle(0).Radian();
        pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/pabi_pose", 100);
    }

    // spin the queue helper thread
    rosQueueThread = thread(bind(&ForceJointPlugin::QueueThread, this));
}

void ForceJointPlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg, string jointName)
{
    jointAngles[jointName] = _msg->data;
    publishPose();
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

void ForceJointPlugin::OnUpdate(const common::UpdateInfo &_info)
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
}

void ForceJointPlugin::QueueThread()
{
    static const double timeout = 0.01;
    while(nh->ok())
    {
        rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}


