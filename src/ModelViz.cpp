#include "roboy_simulation/ModelViz.hpp"

ModelViz::ModelViz(){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ModelViz",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);
}

void ModelViz::publishTendon(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles) {
//    static bool add = true;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    char tendonnamespace[20];
    sprintf(tendonnamespace, "tendon_%d", ID);
    line_strip.ns = tendonnamespace;
    line_strip.action = visualization_msgs::Marker::ADD;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.003;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.lifetime = ros::Duration(0);

    for (uint muscle = 0; muscle < sim_muscles->size(); muscle++) {
        line_strip.points.clear();
        line_strip.id = message_counter++;
        for (uint i = 0; i < (*sim_muscles)[muscle]->viaPoints.size(); i++) {
            geometry_msgs::Point p;
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.z;
            line_strip.points.push_back(p);
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.z;
            line_strip.points.push_back(p);
        }
        marker_visualization_pub.publish(line_strip);
    }
}

void ModelViz::publishForce(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles) {
//    static bool add = true;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char forcenamespace[20];
    sprintf(forcenamespace, "force_%d", ID);
    arrow.ns = forcenamespace;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration(0);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.01;

    arrow.action = visualization_msgs::Marker::ADD;

    for (uint muscle = 0; muscle < sim_muscles->size(); muscle++) {
        for (uint i = 0; i < (*sim_muscles)[muscle]->viaPoints.size(); i++) {
            // actio
            arrow.id = message_counter++;
            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            geometry_msgs::Point p;
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->prevForcePoint.z;
            arrow.points.push_back(p);
            p.x += (*sim_muscles)[muscle]->viaPoints[i]->prevForce.x * 0.001; // show fraction of force
            p.y += (*sim_muscles)[muscle]->viaPoints[i]->prevForce.y * 0.001;
            p.z += (*sim_muscles)[muscle]->viaPoints[i]->prevForce.z * 0.001;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
            // reactio
            arrow.id = message_counter++;
            arrow.color.r = 1.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            p.x = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.x;
            p.y = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.y;
            p.z = (*sim_muscles)[muscle]->viaPoints[i]->nextForcePoint.z;
            arrow.points.push_back(p);
            p.x += (*sim_muscles)[muscle]->viaPoints[i]->nextForce.x * 0.001;
            p.y += (*sim_muscles)[muscle]->viaPoints[i]->nextForce.y * 0.001;
            p.z += (*sim_muscles)[muscle]->viaPoints[i]->nextForce.z * 0.001;
            arrow.points.push_back(p);
            marker_visualization_pub.publish(arrow);
        }
    }
}


void ModelViz::publishModel(const string robot_namespace, physics::LinkPtr parent_link, bool child_link){
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "world";
    char modelnamespace[20];
    sprintf(modelnamespace, "model_%d", ID);
    mesh.ns = modelnamespace;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 0.001;
    mesh.scale.y = 0.001;
    mesh.scale.z = 0.001;
    mesh.lifetime = ros::Duration(0);
    mesh.header.stamp = ros::Time::now();
    mesh.action = visualization_msgs::Marker::ADD;

    if(!child_link) { // parent_link is top link and connected to world frame
        mesh.id = message_counter++;
        math::Pose pose = parent_link->GetWorldPose();
        pose.rot.Normalize();
        mesh.pose.position.x = pose.pos.x;
        mesh.pose.position.y = pose.pos.y;
        mesh.pose.position.z = pose.pos.z;
        mesh.pose.orientation.x = pose.rot.x;
        mesh.pose.orientation.y = pose.rot.y;
        mesh.pose.orientation.z = pose.rot.z;
        mesh.pose.orientation.w = pose.rot.w;
        char meshpath[200];
        sprintf(meshpath,"package://roboy_models/%s/meshes/CAD/%s.stl",
                robot_namespace.c_str(), parent_link->GetName().c_str() );
        mesh.mesh_resource = meshpath;
        marker_visualization_pub.publish(mesh);
    }
    physics::Link_V child_links = parent_link->GetChildJointsLinks();
    if (child_links.empty()) {
        return;
    } else {
        for (auto child_link:child_links) { // each child relative pose to parent
            mesh.id = message_counter++;
            math::Pose pose = child_link->GetWorldPose();
            pose.rot.Normalize();
            mesh.pose.position.x = pose.pos.x;
            mesh.pose.position.y = pose.pos.y;
            mesh.pose.position.z = pose.pos.z;
            mesh.pose.orientation.x = pose.rot.x;
            mesh.pose.orientation.y = pose.rot.y;
            mesh.pose.orientation.z = pose.rot.z;
            mesh.pose.orientation.w = pose.rot.w;
            char meshpath[200];
            sprintf(meshpath,"package://roboy_models/%s/meshes/CAD/%s.stl",
                    robot_namespace.c_str(), child_link->GetName().c_str() );
            mesh.mesh_resource = meshpath;
            marker_visualization_pub.publish(mesh);
            publishModel(robot_namespace, child_link, true);
        }
    }
}
