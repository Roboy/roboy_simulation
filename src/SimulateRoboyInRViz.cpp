#include "roboy_simulation/SimulateRoboyInRViz.hpp"
#include <math.h>
#include <Eigen/src/Core/Matrix.h>

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimulateRoboyInRViz)

SimulateRoboyInRViz::SimulateRoboyInRViz() : ModelPlugin() {
    printf("I'M ALIVE");
}

SimulateRoboyInRViz::~SimulateRoboyInRViz(){}

void SimulateRoboyInRViz::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    printf("\n\n\nSimulateRoboyInRViz - loading --------------- \n\n\n");
    // get the model
    model = _parent;
    // bind the gazebo update function to OnUpdate
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SimulateRoboyInRViz::OnUpdate, this, _1));
    // get all joints and the initial pose
    initPose = model->GetWorldPose();

    // Init ros if it is has not been initialized
    if(!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "Roboy_simplified_moveable"); // name
    }

    // Create ros node
    nh = ros::NodeHandlePtr(new ros::NodeHandle("roboy"));
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    external_force_sub = nh->subscribe("/roboy/external_Force", 1, &SimulateRoboyInRViz::ApplyExternalForce, this);
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/pose", 100);
}

void SimulateRoboyInRViz::ApplyExternalForce(const roboy_communication_simulation::ExternalForce::ConstPtr &msg) {
    /** "Displplaying external force in rviz"*/
    Vector3d force3d(msg->f_x, msg->f_y, msg->f_z);
    Vector3d world_pos3d(msg->x, msg->y, msg->z);
    publishRay(world_pos3d, force3d, "world", "model", 0, COLOR(1,0,1,1), 0 ); // got its own node handler nh

    /** Applying force in simulation*/
    physics::LinkPtr link = model->GetChildLink(msg->name);
    if (link != nullptr) {
        math::Vector3 force(msg->f_x, msg->f_y, msg->f_z);
        math::Vector3 world_pos(msg->x, msg->y, msg->z);
        link->AddForceAtWorldPosition(force, world_pos);
    }
}

void SimulateRoboyInRViz::publishPose()
{
    roboy_communication_middleware::Pose msg;
    int counter = 1;
    for(auto link:model->GetLinks()){
        math::Pose p = link->GetWorldPose();
        /** rviz message*/
        //since in CAD folder no "neck_spinal.stl" specified but model.sdf contains it -> have to ignore it in order to make rviz work
        //for now: workaround / dirty hack -> make prettier or adapt model files
        if (link->GetName() == "neck_spinal") {
            continue;
        }
        Vector3d eigenpos(p.pos.x, p.pos.y, p.pos.z);
        Quaterniond eigenrot(p.rot.w, p.rot.x, p.rot.y, p.rot.z);
        // namespace not important -> rviz separates in certain namespaces, choose same one and you're fine
        // got its own node handler nh
        publishMesh( "roboy_models", "Roboy_simplified_moveable/meshes/CAD", (link->GetName() +".stl").c_str(), eigenpos, eigenrot,
                0.001, "world", "model", counter, 0); // keep message id the same for same obj (message id #0-> force)
        counter ++;

        /** second message construction*/
        msg.name.push_back(link->GetName());
        msg.x.push_back(p.pos.x);
        msg.y.push_back(p.pos.y);
        msg.z.push_back(p.pos.z);
        p.rot.Normalize();
        msg.qx.push_back(p.rot.x);
        msg.qy.push_back(p.rot.y);
        msg.qz.push_back(p.rot.z);
        msg.qw.push_back(p.rot.w);
    }
    /** publish second message*/
    pose_pub.publish(msg);
}


void SimulateRoboyInRViz::OnUpdate(const common::UpdateInfo &_info)
{
    //TODO: needed?
    // this->simulate(model->GetWorld())
    //for each link: publish pose
    publishPose();
}
