#pragma once

// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "roboy_simulation/muscle/IMuscle.hpp"

using namespace gazebo;
using namespace std;

class MyoMuscleVisualization{
public:
    MyoMuscleVisualization();

    void publishTendon(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishForce(vector<boost::shared_ptr<roboy_simulation::IMuscle>> *sim_muscles);

    void publishModel(const string robot_namespace, physics::LinkPtr link);

protected:
    ros::NodeHandlePtr nh;
    uint message_counter;
    ros::Publisher marker_visualization_pub;
    tf::TransformBroadcaster tf_broadcaster;
    int ID;
};
