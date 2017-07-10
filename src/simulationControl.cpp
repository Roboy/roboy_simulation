#include "roboy_simulation/simulationControl.hpp"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
physics::ModelPtr modelControl;

SimulationControl::SimulationControl(){
    // Create a new transport node
    node = transport::NodePtr(new transport::Node());

    // Initialize the node with the world name
    node->Init("simulationControl");

    // Create a publisher on the ~/physics topic
    transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");

    // set the physic params
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::ODE);
    physicsMsg.set_max_step_size(0.0003);
    physicsPub->Publish(physicsMsg);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "SimulationControl", ros::init_options::NoSigintHandler);
    }

    resetPub = node->Advertise<gazebo::msgs::WorldControl>("/gazebo/default/world_control");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    reset_world_srv = nh->advertiseService("/roboy/reset_world", &SimulationControl::resetWorld, this);
    sim_control_sub = nh->subscribe("/roboy/sim_control", 1, &SimulationControl::simulationControl, this);
    recording_control_sub = nh->subscribe("/roboy/recording_control", 1, &SimulationControl::recordingControl, this);
}

SimulationControl& SimulationControl::getInstance() {
    static SimulationControl instance;
    return instance;
}

physics::WorldPtr SimulationControl::loadWorld(string worldName){
    return gazebo::loadWorld(worldName);
}

physics::ModelPtr SimulationControl::loadModel(physics::WorldPtr world, string modelName){
    world->InsertModelFile(modelName);
    physics::ModelPtr model = nullptr;
    // wait until the model is loaded
    int modelCountBefore = world->GetModelCount();
    int retry = 0;
    while (world->GetModelCount() == modelCountBefore) {
        gazebo::runWorld(world, 100);
        gazebo::common::Time::MSleep(100);
        retry++;
        if (retry > 1000)
            break;
    }
    if (world->GetModelCount() == modelCountBefore + 1) {
        ROS_INFO("Successfully inserted model");
        model = world->GetModel("legs_with_upper_body_simplified");
    } else {
        ROS_WARN("Failed inserting model");
    }
    return model;
}


void SimulationControl::simulate(physics::WorldPtr world) {
    if (!paused) {
        gazebo::sensors::run_once(true);
        if (slow_motion) {
            gazebo::runWorld(world, 1);
            ros::Duration d(0.05);
            d.sleep();
        } else {
            gazebo::runWorld(world, 100);
        }
    }

    if (start_time != 0 && stop_time != 0) {
        // If there are start_time and stop_time defined (not 0),
        // it means rosbag recording should start and stop at those times.
        uint32_t now = static_cast<uint32_t>(world->GetSimTime().Double() * 1000.0);
        if (!recording && now >= start_time) {
            startRecording();
        }
        else if (recording && now >= stop_time) {
            stopRecording();
        }
    }
}

bool SimulationControl::resetWorld(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    gazebo::msgs::WorldControl w_ctrl;
    w_ctrl.mutable_reset()->set_all(true);
    resetPub->Publish(w_ctrl);
    res.success = true;
    res.message = "resetting worlds";
    resetRecordingTime();
    return true;
}

void SimulationControl::initializeInterActiveMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                               physics::ModelPtr model, int roboyID) {
    paused = false;
    for (auto link:model->GetLinks()) {
        visualization_msgs::InteractiveMarker int_marker;
        string link_name = link->GetName();
        int_marker.header.frame_id = "world";
        int_marker.header.stamp = ros::Time::now();
        math::Pose pose = link->GetWorldPose();
        int_marker.pose.position.x = pose.pos.x;
        int_marker.pose.position.y = pose.pos.y;
        int_marker.pose.position.z = pose.pos.z;
        int_marker.scale = 0.1;
        int_marker.name = link_name;
        int_marker.description = link_name;

        visualization_msgs::InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        server->insert(int_marker);
        server->setCallback(int_marker.name, &processFeedback);
        server->applyChanges();
    }
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        math::Quaternion q(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                           feedback->pose.orientation.z);
        math::Vector3 pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
        math::Pose pose(pos, q);
        modelControl->GetLink(feedback->marker_name)->SetWorldPose(pose);
    }
    // update all markers except for the one we are currently modifying
    for (auto link:modelControl->GetLinks()) {
        if (link->GetName() == feedback->marker_name)
            continue;
        visualization_msgs::InteractiveMarker marker;
        interactive_marker_server->get(link->GetName(), marker);
        math::Pose pose = link->GetWorldPose();
        marker.pose.position.x = pose.pos.x;
        marker.pose.position.y = pose.pos.y;
        marker.pose.position.z = pose.pos.z;
        marker.pose.orientation.w = pose.rot.w;
        marker.pose.orientation.x = pose.rot.x;
        marker.pose.orientation.y = pose.rot.y;
        marker.pose.orientation.z = pose.rot.z;
        interactive_marker_server->setPose(marker.name, marker.pose);
    }
    interactive_marker_server->applyChanges();
}

void SimulationControl::simulationControl(const std_msgs::Int32::ConstPtr &msg) {
    switch (msg->data) {
        case Play: {
            paused = false;
            slow_motion = false;
            break;
        }
        case Pause: {
            paused = true;
            break;
        }
        case Rewind: {
            gazebo::msgs::WorldControl w_ctrl;
            w_ctrl.mutable_reset()->set_all(true);
            resetPub->Publish(w_ctrl);
            resetRecordingTime();
            break;
        }
        case Slow_Motion: {
            slow_motion = true;
            break;
        }
        case UpdateInteractiveMarker: {
            for (auto link : modelControl->GetLinks()) {
                visualization_msgs::InteractiveMarker marker;
                interactive_marker_server->get(link->GetName(), marker);
                math::Pose pose = link->GetWorldPose();
                marker.pose.position.x = pose.pos.x;
                marker.pose.position.y = pose.pos.y;
                marker.pose.position.z = pose.pos.z;
                marker.pose.orientation.w = pose.rot.w;
                marker.pose.orientation.x = pose.rot.x;
                marker.pose.orientation.y = pose.rot.y;
                marker.pose.orientation.z = pose.rot.z;
                interactive_marker_server->setPose(marker.name, marker.pose);
            }
            interactive_marker_server->applyChanges();
            break;
        }
        case StartRecording: {
            if (start_time == 0 && stop_time == 0) {
                startRecording();
            }
            else {
                ROS_FATAL_STREAM("Timed recording in progress. Can't record two rosbags simultaneously.");
            }
            break;
        }
        case StopRecording: {
            stopRecording();
            break;
        }
    }
}

void SimulationControl::recordingControl(const roboy_communication_simulation::RecordingControl::ConstPtr &msg) {
    {
        lock_guard<mutex> guard(rosbag_mutex);
        if (recording) {
            ROS_FATAL_STREAM("Already recording to file " << rosbag_filename << ". Can't record two rosbags simultaneously.");
            return;
        }
    }

    ROS_INFO_STREAM("Resetting");
    // Reset world
    gazebo::msgs::WorldControl w_ctrl;
    w_ctrl.mutable_reset()->set_all(true);
    resetPub->Publish(w_ctrl);

    // Initiate recording
    start_time = msg->start_time;
    stop_time = msg->stop_time;
    ROS_INFO_STREAM("Recording will start after " << start_time << "ms and stop after " << stop_time << "ms");
}

void SimulationControl::startRecording() {
    if (!recording) {
        // Search for an unused filename
        struct stat buffer;
        static int i = 0;
        do {
            sprintf(rosbag_filename, rosbag_filename_template, i);
            i++;
        } while (stat (rosbag_filename, &buffer) == 0);

        // Open the rosbag file for recording
        lock_guard<mutex> guard(rosbag_mutex);
        rosbag.open(rosbag_filename, rosbag::bagmode::Write);
        recording = true;
        ROS_INFO_STREAM("Started recording to " << rosbag_filename);
    }
}

void SimulationControl::stopRecording() {
    lock_guard<mutex> guard(rosbag_mutex);
    if (recording) {
        recording = false;
        rosbag.close();
        ROS_INFO_STREAM("Stopped recording to " << rosbag_filename);
    }
    resetRecordingTime();
}

void SimulationControl::resetRecordingTime() {
    start_time = stop_time = 0;
}
