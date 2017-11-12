#pragma once

// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
// messages
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// common definitions
// muscle plugin
#include "roboy_simulation/muscle/IMuscle.hpp"

using namespace gazebo;
using namespace std;

class ModelViz{
public:
    ModelViz();

    void publishTendon(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishForce(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishModel(const string robot_namespace, physics::LinkPtr parent_link, bool child_link);

protected:
    ros::NodeHandlePtr nh;
    int ID;
    uint message_counter;
    bool visualizeTendon = false, visualizeForce = false, visualizeMesh = false;
    ros::Publisher marker_visualization_pub;
private:
    ros::Subscriber visualization_control_sub;
    tf::TransformBroadcaster tf_broadcaster;
};
