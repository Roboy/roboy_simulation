#include "roboy_simulation/ForceLinkPlugin.hpp"

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ForceLinkPlugin)

ForceLinkPlugin::ForceLinkPlugin() : ModelPlugin() {}

ForceLinkPlugin::~ForceLinkPlugin(){}

void ForceLinkPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
      // get the model
    model = _parent;
    // bind the gazebo update function to OnUpdate
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ForceLinkPlugin::OnUpdate, this, _1));
    
    // Init ros if it is has not been initialized
    if(!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "Roboy");
    }
    // Create ros node
    nh = ros::NodeHandlePtr(new ros::NodeHandle("roboy"));
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();
    pose_sub = nh->subscribe("/roboy/simulation/external_pose", 1, &ForceLinkPlugin::PoseCommand, this);
    pose_service = nh->advertiseService("roboy/simulation/print_poses", &ForceLinkPlugin::PrintLinkPose, this);

    // get all links and the initial pose
    physics::Link_V linkVector = model->GetLinks();
    initPose = model->GetWorldPose();

    for(auto link = linkVector.begin(); link != linkVector.end(); link++)
    {
	    string linkname = (*link)->GetName();
        linkNames.push_back(linkname);
	    // linkPoses[linkname] = (*link)->GetWorldPose();
        linkPoses[linkname] = math::Pose(math::Vector3(0,0,0), math::Quaternion(1,0,0,0));
    }
}

void ForceLinkPlugin::PoseCommand(const roboy_communication_middleware::PoseConstPtr &msg){
    cout << "Got a command!" << endl;
    for(uint i=0;i<msg->name.size();i++){
	    math::Vector3 pos(msg->x[i], msg->y[i], msg->z[i]);
	    math::Quaternion rot(msg->qx[i], msg->qy[i], msg->qz[i], msg->qw[i]);
	    math::Pose pose(pos, rot);
        linkPoses[msg->name[i]] = pose;
    }
}

void ForceLinkPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    // make the model stationary
    model->SetWorldPose(initPose);
    // adjust the force so that the link goes towards the wanted position
    for(auto linkName = linkNames.begin(); linkName != linkNames.end(); linkName++)
    {
        physics::LinkPtr link = model->GetLink(*linkName);
        // we assume that the user does not send a 0,0,0 position as destination therefore we can use this a zero vector
        // to stop the link
        if(IsVectorZero(linkPoses[*linkName].pos))
        {
            link->SetForce(math::Vector3(0,0,0));
        }
        // otherwise we have to calculate the distance between the wanted position and the current position
        else
        {
            double distance = link->GetWorldPose().pos.Distance(linkPoses[*linkName].pos);
            if(distance > 1)
            {
                link->SetForce((linkPoses[*linkName].pos - link->GetWorldPose().pos).Normalize() * 10);
            }
            else
            {
                cout << "Setting force to 0 again of link:" << *linkName << endl;
                linkPoses[*linkName].Set(math::Vector3(0,0,0), math::Quaternion(1,0,0,0));
                link->SetForce(math::Vector3(0,0,0));
            }
        }
        //model->GetLink(*linkName)->SetWorldPose(linkPoses[*linkName]);
    }
}

bool ForceLinkPlugin::IsVectorZero(math::Vector3 v)
{
    return v.x == 0 && v.y == 0 && v.z == 0;
}

bool ForceLinkPlugin::PrintLinkPose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;
    res.message = "what";
    cout << model->GetLink("LeftHandFinger1")->GetWorldPose().pos << endl;
    cout << model->GetLink("LeftHandFinger1")->GetWorldPose().rot << endl;
}
