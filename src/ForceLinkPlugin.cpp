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
    pose_sub = nh->subscribe("/roboy/simulation/" + _parent->GetName() + "/external_pose", 1, &ForceLinkPlugin::PoseCommand, this);

    // get all links and the initial pose
    physics::Link_V linkVector = model->GetLinks();
    initPose = model->GetWorldPose();

    for(auto link = linkVector.begin(); link != linkVector.end(); link++)
    {
	    string linkname = (*link)->GetName();
        linkNames.push_back(linkname);
	    linkPoses[linkname] = (*link)->GetWorldPose();
        //linkPoses[linkname] = math::Pose(math::Vector3(0,0,0), math::Quaternion(1,0,0,0));
    }
}

void ForceLinkPlugin::PoseCommand(const roboy_communication_middleware::PoseConstPtr &msg){
    for(uint i=0;i<msg->name.size();i++){
	    math::Vector3 pos(msg->x[i], msg->y[i], msg->z[i]);
	    math::Quaternion rot(msg->qw[i], msg->qx[i], msg->qy[i], msg->qz[i]);
	    math::Pose pose(pos, rot);
        linkPoses[msg->name[i]] = pose;
    }
}

void ForceLinkPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    // make the model stationary
    // model->SetWorldPose(initPose);
    // set the pose of the links to the poses in the map
    for(auto linkName = linkNames.begin(); linkName != linkNames.end(); linkName++)
    {
        model->GetLink(*linkName)->SetForce(math::Vector3(0,0,0));
        model->GetLink(*linkName)->SetWorldPose(linkPoses[*linkName]);
    }
}
