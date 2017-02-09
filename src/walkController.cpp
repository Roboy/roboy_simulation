#include "roboy_simulation/walkController.hpp"

int WalkController::roboyID_generator = 0;

WalkController::WalkController() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkController",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    force_torque_ankle_left_sub = nh->subscribe("/roboy/force_torque_ankle_left", 1,
                                                &WalkController::finite_state_machine, this);
    force_torque_ankle_right_sub = nh->subscribe("/roboy/force_torque_ankle_right", 1,
                                                 &WalkController::finite_state_machine, this);

    roboyID_pub = nh->advertise<std_msgs::Int32>("/roboy/id",1);
    abort_pub = nh->advertise<roboy_simulation::Abortion>("/roboy/abort", 1000);
    motor_control_sub = nh->subscribe("/roboy/motor_control", 100, &WalkController::motorControl, this);

    imu_pub = nh->advertise<roboy_simulation::IMU>("/roboy/imu", 100);

    joint_pub = nh->advertise<roboy_simulation::Joint>("/roboy/joint", 12);

    body_pub = nh->advertise<roboy_simulation::BodyPart>("/roboy/body",13);

    COM_pub = nh->advertise<roboy_simulation::COM>("/roboy/COM",1);

    body_pub = nh->advertise<roboy_simulation::BodyPart>("/roboy/body", 13);

    input_pub = nh->advertise<roboy_simulation::Input>("/roboy/inputV", 12);

    roboyID = roboyID_generator++;
    ID = roboyID;
    char topic[200];
    sprintf(topic, "/roboy%d/controller_parameters", roboyID);
    control_parameters_srv = nh->advertiseService(topic, &WalkController::updateControllerParameters, this);

    sprintf(topic, "/roboy%d/energies", roboyID);
    energies_srv = nh->advertiseService(topic, &WalkController::energiesService, this);

    params.resize(TOTAL_NUMBER_CONTROLLER_PARAMETERS);

    leg_state[LEG::LEFT] = Stance;
    leg_state[LEG::RIGHT] = Swing;

    // Generate the Gaussian kernel for IMU data low-pass filtering
    ROS_INFO_STREAM("gaussian_kernel size and shape:");

    // Set standard deviation to 3.0
    double sigma = 3.0;
    double r, s = 2.0 * sigma * sigma;

    double sum = 0.0;
    int half_size = ACCEL_WIN_SIZE / 2.0;
    // If ACCEL_WIN_SIZE is odd, add one extra coefficient in the kernel
    int add_coeff = (ACCEL_WIN_SIZE % 2 == 0 ? 0 : 1);

    // Generate kernel
    for(int i = -half_size; i < half_size + add_coeff; i++)
    {
        r = abs(i);
        double coeff = (exp(-(r*r)/s))/(M_PI * s);
        gaussian_kernel.push_back(coeff);
        sum += coeff;
    }

    // Normalize the kernel
    for(int i = 0; i < gaussian_kernel.size(); i++)
    {
        gaussian_kernel[i] /= sum;
        // Visualize the shape of the kernel
        ROS_INFO_STREAM(std::string(200 * gaussian_kernel[i], '#'));
    }

    if (gaussian_kernel.size() != ACCEL_WIN_SIZE) {
        // This should never happen, but if it happens, give a warning
        ROS_WARN_STREAM("gaussian_kernel size doesn't match ACCEL_WIN_SIZE!");
    }
}

WalkController::~WalkController() {
}

void WalkController::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
    ROS_INFO("Loading WalkController plugin");
    // Save pointers to the model
    parent_model = parent_;
    sdf = sdf_;

    // Error message if the model couldn't be found
    if (!parent_model) {
        ROS_ERROR_STREAM_NAMED("loadThread", "parent model is NULL");
        return;
    }

    // Check that ROS has been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    // Get namespace for nodehandle
    if (sdf_->HasElement("robotNamespace")) {
        robot_namespace = sdf_->GetElement("robotNamespace")->Get<std::string>();
    } else {
        robot_namespace = parent_model->GetName(); // default
    }

    // Get robot_description ROS param name
    if (sdf_->HasElement("robotParam")) {
        robot_description = sdf_->GetElement("robotParam")->Get<std::string>();
    } else {
        robot_description = "robot_description"; // default
    }

    // Get all link and joint names
    physics::Link_V links = parent_model->GetLinks();
    for (auto link : links) {
        string link_name = link->GetName();
        link_names.push_back(link_name);
        acceleration_windows[link_name] = vector<math::Vector3>(ACCEL_WIN_SIZE, math::Vector3(0, 0, 0));
    }

    physics::Joint_V joints = parent_model->GetJoints();
    for (auto joint : joints) {
        joint_names.push_back(joint->GetName());
    }

    physics::PhysicsEnginePtr physics_engine = parent_model->GetWorld()->GetPhysicsEngine();

    // Get the Gazebo solver type
    std::string solver_type = boost::any_cast<std::string>(physics_engine->GetParam("solver_type"));

    // Give the global CFM (Constraint Force Mixing) a positive value (default 0) making the model a bit less stiff to avoid jittering
    // Positive CFM allows links to overlap each other so the collisions aren't that hard
    physics_engine->SetParam("cfm", 0.005);
    double cfm = boost::any_cast<double>(physics_engine->GetParam("cfm"));

    // Get the global ERP (Error Reduction Parameter) which is similar to CFM but is about joints
    double erp = boost::any_cast<double>(physics_engine->GetParam("erp"));

    ROS_INFO_STREAM("Simulation max_step_size: " << gazebo_max_step_size << " s");
    ROS_INFO_STREAM("Solver type: " << solver_type);
    ROS_INFO_STREAM("Global ERP: " << erp);
    ROS_INFO_STREAM("Global CFM: " << cfm);

    // List all models in the world
    ROS_INFO_STREAM("List of models in the world:");
    physics::Model_V models = parent_model->GetWorld()->GetModels();
    for (auto model : models) {
        std::string model_name = model->GetName();
        ROS_INFO_STREAM("Model \"" << model_name << "\" links:");
        physics::Link_V links = model->GetLinks();
        // List all links of the model
        for (auto link : links) {
            std::string link_name = link->GetName();
            ROS_INFO_STREAM("  " << link_name);

            // Modify and list the friction parameters of the link
            physics::Collision_V collisions = link->GetCollisions();
            for (auto collision : collisions) {
                physics::FrictionPyramidPtr friction = collision->GetSurface()->FrictionPyramid();
                friction->SetMuPrimary(100);
                friction->SetMuSecondary(50);
                friction->SetMuTorsion(10);
                friction->SetPatchRadius(10);
                friction->SetUsePatchRadius(true);
                friction->SetElasticModulus(0.01);
                //friction->SetPoissonsRatio(5);

                ROS_INFO_STREAM("    Mu_1:            " << friction->MuPrimary());
                ROS_INFO_STREAM("    Mu_2:            " << friction->MuSecondary());
                ROS_INFO_STREAM("    Mu_Torsion:      " << friction->MuTorsion());
                ROS_INFO_STREAM("    Patch radius:    " << friction->PatchRadius());
                ROS_INFO_STREAM("    Elastic modulus: " << friction->ElasticModulus());
                ROS_INFO_STREAM("    Poisson's ratio: " << friction->PoissonsRatio());
            }
        }
    }

    gazebo_max_step_size = physics_engine->GetMaxStepSize();

    // Get the Gazebo simulation period
    ros::Duration gazebo_period = ros::Duration(gazebo_max_step_size);

    // Decide the plugin control period
    control_period = ros::Duration(0.1);
    if (sdf_->HasElement("controlPeriod")) {
        control_period = ros::Duration(sdf_->Get<double>("controlPeriod"));

        ROS_INFO_STREAM("Desired controller update period: " << control_period);
        // Check the period against the simulation period
        if (control_period < gazebo_period) {
            ROS_ERROR_STREAM("Desired controller update period (" << control_period
                                                                  << " s) is faster than the gazebo simulation period ("
                                                                  <<
                                                                  gazebo_period << " s).");
        } else if (control_period > gazebo_period) {
            ROS_WARN_STREAM("Desired controller update period (" << control_period
                                                                 << " s) is slower than the gazebo simulation period ("
                                                                 <<
                                                                 gazebo_period << " s).");
        }
    } else {
        control_period = gazebo_period;
        ROS_DEBUG_STREAM("Control period not found in URDF/SDF, defaulting to Gazebo period of "
                                 << control_period);
    }

    // Create the controller manager
//    ROS_INFO_STREAM_NAMED("ros_control_plugin", "Loading controller_manager");
//        cm = new controller_manager::ControllerManager(this, *nh);

    // Initialize the emergency stop code.
    e_stop_active = false;
    last_e_stop_active = false;
    if (sdf_->HasElement("eStopTopic")) {
        const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
        e_stop_sub = nh->subscribe(e_stop_topic, 1, &WalkController::eStopCB, this);
    }

    ROS_INFO("Parsing myoMuscles");
    if (!parseMyoMuscleSDF(sdf_->ToString(""), myoMuscles))
        ROS_WARN("ERROR parsing myoMuscles, check your sdf file.");
    numberOfMyoMuscles = myoMuscles.size();
    ROS_INFO("Found %d MyoMuscles in sdf file", numberOfMyoMuscles);

    // class laoder for loading muscle plugins
    class_loader.reset(new pluginlib::ClassLoader<roboy_simulation::IMuscle>
                               ("roboy_simulation",
                                "roboy_simulation::IMuscle"));

    sim_muscles.clear();
    for (uint muscle = 0; muscle < myoMuscles.size(); muscle++) {
        try {
            ROS_INFO("Loading Muscle Plugin for %s",
                           myoMuscles[muscle].name.c_str());
            sim_muscles.push_back(class_loader->createInstance("roboy_simulation::IMuscle"));
            sim_muscles.back()->Init(myoMuscles[muscle]);

            muscles_spanning_joint[sim_muscles[muscle]->spanningJoint->GetName()].push_back(muscle);

            // initialize the queue for delayed activities depending on the spanning joint
            // hip muscles: 5ms, knee muscles 10ms, ankle muscles 20ms
            if(sim_muscles[muscle]->spanningJoint->GetName().find("groin")!=string::npos){
                for(uint i=0;i<5e-03/gazebo_max_step_size;i++){
                    activity[sim_muscles[muscle]->name].push_back(0.0);
                }
            }else if(sim_muscles[muscle]->spanningJoint->GetName().find("knee")!=string::npos) {
                for (uint i = 0; i < 10e-03 / gazebo_max_step_size; i++) {
                    activity[sim_muscles[muscle]->name].push_back(0.0);
                }
            }else if(sim_muscles[muscle]->spanningJoint->GetName().find("ankle")!=string::npos) {
                for (uint i = 0; i < 20e-03 / gazebo_max_step_size; i++) {
                    activity[sim_muscles[muscle]->name].push_back(0.0);
                }
            }

            a[sim_muscles[muscle]->name] = 0.0;
        }
        catch (pluginlib::PluginlibException &ex) {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }

    hip_CS = CoordSys(new CoordinateSystem(parent_model->GetLink("hip")));

    updateFootDisplacementAndVelocity();

    calculateCOM(POSITION, initial_center_of_mass_height);

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&WalkController::Update, this));

    desiredAngles.clear();
    jointPIDs.clear();

    for (auto joint_name : joint_names) {
      physics::JointPtr thisJoint = parent_model->GetJoint(joint_name);
      desiredAngles[joint_name] = 0;
      jointPIDs[joint_name] = gazebo::common::PID(15,0,10,outputMax,outputMin,outputMax,outputMin);
    }

    ROS_INFO("WalkController ready");
}

void WalkController::Update() {
    static long unsigned int counter = 0;
    // Get the simulation time and period
    gz_time_now = parent_model->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;

    updateFootDisplacementAndVelocity();

    if(getLegInState(Stance)==LEG::NONE) {
        ROS_WARN_THROTTLE(1.0, "legs are not touching the ground");
    }

    if(control) {
        // Compute the controller commands
        updateTargetFeatures();
        updateMuscleForces();
        updateMuscleActivities();
        updateEnergies();
    }

    // Check if we should update the controllers
    if (sim_period >= control_period) {
        // Store this simulation time
        last_update_sim_time_ros = sim_time_ros;

        // Update the robot simulation with the state of the gazebo model
        readSim(sim_time_ros, sim_period);
    }

    // Update the gazebo model with the result of the controller
    // computation
    writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);
    last_write_sim_time_ros = sim_time_ros;

    // publish every now and then
    if (counter % 100 == 0) {
        message_counter = 1000;
        if (visualizeTendon)
            publishTendon(&sim_muscles);
        if (visualizeForce)
            publishForce(&sim_muscles);
        if (visualizeCOM)
            publishCOM(center_of_mass);
        if (visualizeEstimatedCOM)
            publishEstimatedCOM();
        if (visualizeMomentArm)
            publishMomentArm(&sim_muscles);
        if (visualizeMesh)
            publishModel(parent_model->GetLink("hip"), false);
        if (visualizeStateMachineParameters)
            publishStateMachineParameters(center_of_mass, foot_sole_global, hip_CS, params);

        publishIMUs();
        publishPositionsAndMasses();
        publishCoordinateSystems(parent_model->GetLink("hip"), ros::Time::now(), false);
        publishSimulationState(params, gz_time_now);
        publishID();
        publishLegState(leg_state);
        publishCOMmsg();
        controlJoints();
    }

    checkAbort();
}

void WalkController::readSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("read simulation");
    // update muscle plugins
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        for(int i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++){
            math::Pose linkPose = sim_muscles[muscle]->viaPoints[i]->link->GetWorldPose();
            sim_muscles[muscle]->viaPoints[i]->linkPosition = linkPose.pos;
            sim_muscles[muscle]->viaPoints[i]->linkRotation = linkPose.rot;
        }
        sim_muscles[muscle]->Update(time, period);
    }
}

void WalkController::writeSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("write simulation");
    // apply the calculated forces
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        for(int i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++){
            std::shared_ptr<roboy_simulation::IViaPoints> vp = sim_muscles[muscle]->viaPoints[i];
            if(vp->prevForcePoint.IsFinite() && vp->nextForcePoint.IsFinite() ) {
                vp->link->AddForceAtWorldPosition(vp->prevForce, vp->prevForcePoint);
                vp->link->AddForceAtWorldPosition(vp->nextForce, vp->nextForcePoint);
            }
        }
    }
}

void WalkController::Reset() {
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros = ros::Time();
    last_write_sim_time_ros = ros::Time();

    // Reset the recorded acceleration data
    for (auto it = acceleration_windows.begin(); it != acceleration_windows.end(); it++) {
        it->second.clear();
        it->second.resize(ACCEL_WIN_SIZE, math::Vector3(0.0, 0.0, 0.0));
    }
}

void WalkController::finite_state_machine(const roboy_simulation::ForceTorque::ConstPtr &msg) {
    // check what state the leg is currently in
    bool state_transition = false;

    switch (leg_state[msg->leg]) {
        case Stance: {
            if(initial_contact[msg->leg]){ // calculate base velocity based on the foot position
                initial_contact[msg->leg] = false;
                initial_contact_time[msg->leg] = gz_time_now.Float();
                initial_contact_pos[msg->leg] = foot_sole_global[msg->leg];
                if(msg->leg == LEG::LEFT){
                    v_base = (parent_model->GetLink(FOOT[msg->leg])->GetWorldPose().pos
                              - initial_contact_pos[LEG::RIGHT]).GetLength()
                             /(initial_contact_time[msg->leg]-initial_contact_time[LEG::RIGHT]);
                }else if(msg->leg == LEG::RIGHT) {
                    v_base = (parent_model->GetLink(FOOT[msg->leg])->GetWorldPose().pos
                              - initial_contact_pos[LEG::LEFT]).GetLength()
                             / (initial_contact_time[msg->leg] - initial_contact_time[LEG::LEFT]);
                }
            }

            if (params[d_s+msg->leg] < params[d_lift] || (leg_state[LEG::LEFT] == Stance && leg_state[LEG::RIGHT] ==
                                                                                                   Stance)) {
                state_transition = true;
                ROS_INFO("ds: %f d_lift: %f", params[d_s+msg->leg] , params[d_lift]);
            }
            break;
        }
        case Lift_off: {
            initial_contact[msg->leg] = true;
            double force_norm = sqrt(pow(msg->force.x, 2.0) + pow(msg->force.y, 2.0) + pow(msg->force.z, 2.0));
            if (force_norm < params[F_contact]) {
                state_transition = true;
                ROS_INFO("force_norm: %f F_contact: %f", force_norm, params[F_contact]);
            }
            break;
        }
        case Swing: {
            if (params[d_s+msg->leg] > params[d_prep]) {
                state_transition = true;
                ROS_INFO("d_s: %f d_prep: %f", params[d_s+msg->leg] , params[d_prep]);
            }
            break;
        }
        case Stance_Preparation: {
            double force_norm = sqrt(pow(msg->force.x, 2.0) + pow(msg->force.y, 2.0) + pow(msg->force.z, 2.0));
            if (force_norm > params[F_contact]) {
                state_transition = true;
                ROS_INFO("force_norm: %f F_contact: %f", force_norm, params[F_contact]);
            }
            break;
        }
    }

    if (state_transition) {
        LEG_STATE new_state = NextState(leg_state[msg->leg]);
        ROS_INFO("%s state transition \t%s \t-> \t%s", LEG_NAMES_STRING[msg->leg],
                 LEG_STATE_STRING[leg_state[msg->leg]],
                 LEG_STATE_STRING[new_state] );
        leg_state[msg->leg] = new_state;
    }

    if(visualizeForceTorqueSensors){
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        char forcetorquenamespace[20];
        sprintf(forcetorquenamespace, "forceTorqueSensors_%d", roboyID);
        arrow.ns = forcetorquenamespace;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.lifetime = ros::Duration();
        arrow.scale.x = 0.005;
        arrow.scale.y = 0.03;
        arrow.scale.z = 0.03;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.color.a = 1.0;
        arrow.id = 10000 + msg->leg;
        arrow.header.stamp = ros::Time::now();
        arrow.points.clear();
        geometry_msgs::Point p;
        math::Vector3 pos = parent_model->GetJoint(msg->joint)->GetWorldPose().pos;
        p.x = pos.x;
        p.y = pos.y;
        p.z = pos.z;
        arrow.points.push_back(p);
        p.x += msg->force.x;
        p.y += msg->force.y;
        p.z += msg->force.z;
        arrow.points.push_back(p);
        marker_visualization_pub.publish(arrow);
    }
}

LEG_STATE WalkController::NextState(LEG_STATE s) {
    LEG_STATE newstate;
    switch (s) {
        case Stance:
            newstate = Lift_off;
            break;
        case Lift_off:
            newstate = Swing;
            break;
        case Swing:
            newstate = Stance_Preparation;
            break;
        case Stance_Preparation:
            newstate = Stance;
            break;
    }
    return newstate;
}

LEG WalkController::getLegInState(LEG_STATE s) {
    if (leg_state[LEG::LEFT] == s)
        return LEG::LEFT;
    else if (leg_state[LEG::RIGHT] == s)
        return LEG::RIGHT;
    else
        return LEG::NONE;
}

void WalkController::calculateCOM(int type, math::Vector3 &COM) {
    double mass_total = 0;
    COM = math::Vector3(0, 0, 0);
    for (auto link_name:link_names) {
        physics::LinkPtr link = parent_model->GetLink(link_name);
        double m = link->GetInertial()->GetMass();
        mass_total += m;
        switch (type) {
            case POSITION: {
                math::Pose p = link->GetWorldCoGPose();
                COM += p.pos * m;
                break;
            }
            case VELOCITY: {
                math::Vector3 v = link->GetWorldCoGLinearVel();
                COM += v * m;
                break;
            }
        }
    }

    COM /= mass_total;
}

void WalkController::publishEstimatedCOM() {
    // Construct the visualization message for the estimated position of COM
    // illustrated by a sphere
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    char estimatedcomnamespace[40];
    sprintf(estimatedcomnamespace, "EstCOM_%d", ID);
    sphere.ns = estimatedcomnamespace;
    sphere.type = visualization_msgs::Marker::SPHERE;

    // The sphere is transparent light blue
    sphere.color.r = 0.5f;
    sphere.color.g = 0.5f;
    sphere.color.b = 1.0f;
    sphere.color.a = 0.5f;

    sphere.lifetime = ros::Duration(0);

    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;

    sphere.action = visualization_msgs::Marker::ADD;

    sphere.header.stamp = ros::Time::now();
    sphere.points.clear();
    sphere.id = message_counter++;

    // TODO: Fetch the estimation of the position of COM from the neural network

    sphere.pose.position.x = 0.0f; // REPLACE WITH NN ESTIMATION
    sphere.pose.position.y = 0.0f; // REPLACE WITH NN ESTIMATION
    sphere.pose.position.z = 0.0f; // REPLACE WITH NN ESTIMATION

    marker_visualization_pub.publish(sphere);
}

void WalkController::publishIMUs() {
    // Construct the arrow visualization and /roboy/imu messages illustrating the accelerations
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    char imunamespace[20];
    sprintf(imunamespace, "imu_%d", ID);
    arrow.ns = imunamespace;
    arrow.type = visualization_msgs::Marker::ARROW;

    arrow.scale.x = 0.01; // Shaft diameter scale
    arrow.scale.y = 0.03; // Head diameter scale
    arrow.scale.z = 0.03; // Head length scale

    arrow.lifetime = ros::Duration();
    arrow.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;

    roboy_simulation::IMU imu_msg;

    imu_msg.roboyID = roboyID;

    for (auto link_name : link_names) {
        imu_msg.link = link_name.c_str();
        arrow.header.stamp = ros::Time::now();

        physics::LinkPtr link = parent_model->GetLink(link_name);
        math::Pose pose = link->GetWorldCoGPose();

        math::Vector3 filtered_lin_accel = getFilteredLinearAcceleration(link);
        math::Vector3 ang_vel_world = link->GetWorldAngularVel();

        // IMU message (not for rviz visualization)

        imu_msg.lin_accel_world.x = filtered_lin_accel.x;
        imu_msg.lin_accel_world.y = filtered_lin_accel.y;
        imu_msg.lin_accel_world.z = filtered_lin_accel.z;

        imu_msg.ang_vel_world.x = ang_vel_world.x;
        imu_msg.ang_vel_world.y = ang_vel_world.y;
        imu_msg.ang_vel_world.z = ang_vel_world.z;

        // Publish imu and link msgs //
        imu_pub.publish(imu_msg);

        SimulationControl &simcontrol = SimulationControl::getInstance();
        simcontrol.writeRosbagIfRecording("/roboy/imu", imu_msg);

        if (visualizeIMUs) {
            // The acceleration vector starts at the center of gravity of the link
            start_point.x = pose.pos.x;
            start_point.y = pose.pos.y;
            start_point.z = pose.pos.z;

            arrow.id = message_counter++;
            arrow.points.clear();

            arrow.points.push_back(start_point);

            // Linear world acceleration with red color
            arrow.color.r = 1.0f;
            arrow.color.g = 0.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;

            end_point.x = start_point.x + filtered_lin_accel.x * 0.03;
            end_point.y = start_point.y + filtered_lin_accel.y * 0.03;
            end_point.z = start_point.z + filtered_lin_accel.z * 0.03;
            arrow.points.push_back(end_point);
            marker_visualization_pub.publish(arrow);

            arrow.id = message_counter++;
            arrow.points.clear();

            arrow.points.push_back(start_point);

            // Angular world velocity with green color
            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;

            end_point.x = start_point.x + ang_vel_world.x * 0.02;
            end_point.y = start_point.y + ang_vel_world.y * 0.02;
            end_point.z = start_point.z + ang_vel_world.z * 0.02;
            arrow.points.push_back(end_point);
            marker_visualization_pub.publish(arrow);
        }
    }
}

void WalkController::publishPositionsAndMasses() {
    // Publish the position and mass of each link

    roboy_simulation::BodyPart body_msg;
    body_msg.roboyID = roboyID;

    for (auto link_name : link_names) {
        physics::LinkPtr link = parent_model->GetLink(link_name);
        math::Pose pose = link->GetWorldCoGPose();
        body_msg.link = link_name.c_str();

        body_msg.position.x = pose.pos.x;
        body_msg.position.y = pose.pos.y;
        body_msg.position.z = pose.pos.z;

        body_msg.mass = link->GetInertial()->GetMass();

        body_pub.publish(body_msg);

        SimulationControl &simcontrol = SimulationControl::getInstance();
        simcontrol.writeRosbagIfRecording("/roboy/body", body_msg);
    }
}

void WalkController::updateFootDisplacementAndVelocity(){
    // calculate the COM position and velocity
    calculateCOM(POSITION, center_of_mass[POSITION]);
    calculateCOM(VELOCITY, center_of_mass[VELOCITY]);

    // calculate foot_sole position in world frame
    math::Pose foot_pose = parent_model->GetLink(FOOT[LEG::LEFT])->GetWorldPose();
    foot_sole_global[LEG::LEFT] = foot_pose.pos + foot_pose.rot.RotateVector(foot_sole[LEG::LEFT]);
    foot_pose = parent_model->GetLink(FOOT[LEG::RIGHT])->GetWorldPose();
    foot_sole_global[LEG::RIGHT] = foot_pose.pos + foot_pose.rot.RotateVector(foot_sole[LEG::RIGHT]);

    // update coordinate system wrt heading
    hip_CS->UpdateHeading();

    // v_COM is the velocity of center_of_mass projected onto forward direction
    v_COM = center_of_mass[VELOCITY].Dot(hip_CS->X);

    // calculate signed horizontal distance between foot_pos and COM
    for(uint leg=LEG::LEFT; leg<=LEG::RIGHT; leg++) {
        // get the foot velocity
        math::Vector3 foot_vel = parent_model->GetLink(FOOT[leg])->GetWorldCoGLinearVel();

        // calculate the distance and velocity between foot and center of mass in world frame
        d_foot_pos[leg] = foot_sole_global[leg] - center_of_mass[POSITION];
        d_foot_vel[leg] = foot_vel - center_of_mass[VELOCITY];

        // rotate into hip coordinate system
        d_foot_pos[leg] = hip_CS->rot.RotateVector(d_foot_pos[leg]);
        d_foot_vel[leg] = hip_CS->rot.RotateVector(d_foot_vel[leg]);

        // calculate signed sagittal and coronal foot displacement and velocity
        params[d_s+leg] = d_foot_pos[leg].Dot(hip_CS->X);
        params[d_c+leg] = d_foot_pos[leg].Dot(hip_CS->Y);
        params[v_s+leg] = d_foot_vel[leg].Dot(hip_CS->X);
        params[v_c+leg] = d_foot_vel[leg].Dot(hip_CS->Y);
    }
}

void WalkController::updateTargetFeatures() {
    // target velocity
    math::Vector3 v_target(v_forward, 0, 0);
    v_target = hip_CS->rot.RotateVector(v_target);

    math::Pose hip_pose = parent_model->GetLink("hip")->GetWorldPose();

    // trunk
    double theta_trunk = params[theta_trunk_0] + params[k_v] * (v_forward - v_COM);
    double phi_trunk = params[phi_trunk_0] + params[k_h] * (psi_heading - hip_pose.rot.GetYaw());
    // target orientation for the hip
    Q["hip"] = math::Quaternion(phi_trunk, theta_trunk, psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["hip"] = hip_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["hip"] = v_target;
    // target angular velocity is zero
    omega["hip"] = math::Vector3::Zero;

    // thigh left
    physics::LinkPtr thigh_left = parent_model->GetLink("thigh_left");
    double theta_groin_left = params[theta_groin_0+LEG::LEFT]
                              + params[k_p_theta_left+leg_state[LEG::LEFT]] * params[d_s+LEG::LEFT]
                              + params[k_d_theta_left+leg_state[LEG::LEFT]] * (v_forward - params[v_s+LEG::LEFT]);
    double phi_groin_left = params[phi_groin_0+LEG::LEFT]
                            + params[k_p_phi+LEG::LEFT] * params[d_c+LEG::LEFT]
                            - params[k_d_phi+LEG::LEFT] * params[v_c+LEG::LEFT];
    // target orientation for the hip
    Q["thigh_left"] = math::Quaternion(phi_groin_left, theta_groin_left, psi_heading);
    // there is no target position, so we set it to the current position
    P["thigh_left"] = thigh_left->GetWorldPose().pos;
    // there is no target velocity, so we set it to the current velocity
    v["thigh_left"] = thigh_left->GetWorldCoGLinearVel();
    // target angular velocity is zero while stance preparation
    if (leg_state[LEG::LEFT] == Stance_Preparation)
        omega["thigh_left"] = math::Vector3::Zero;
    else  // there is no target
        omega["thigh_left"] = thigh_left->GetWorldAngularVel();

    // thigh right
    physics::LinkPtr thigh_right = parent_model->GetLink("thigh_right");
    double theta_groin_right = params[theta_groin_0+LEG::RIGHT]
                               + params[k_p_theta_right+leg_state[LEG::RIGHT]] * params[d_s+LEG::LEFT]
                               + params[k_d_theta_right+leg_state[LEG::RIGHT]]
                                 * (v_forward - params[v_s+LEG::LEFT]);
    double phi_groin_right = params[phi_groin_0+LEG::RIGHT]
                             + params[k_p_phi+LEG::RIGHT] * params[d_c+LEG::LEFT]
                             - params[k_d_phi+LEG::RIGHT] * params[v_c+LEG::LEFT];
    // target orientation for the hip
    Q["thigh_right"] = math::Quaternion(phi_groin_right, theta_groin_right, psi_heading);
    // there is no target position, so we set it to the current position
    P["thigh_right"] = thigh_right->GetWorldPose().pos;
    // there is no target velocity, so we set it to the current velocity
    v["thigh_right"] = thigh_right->GetWorldCoGLinearVel();
    // target angular velocity is zero while stance preparation
    if (leg_state[LEG::RIGHT] == Stance_Preparation) // target is zero while stance preparation
        omega["thigh_right"] = math::Vector3::Zero;
    else  // there is no target
        omega["thigh_right"] = thigh_right->GetWorldAngularVel();

    // shank left
    physics::LinkPtr shank_left = parent_model->GetLink("shank_left");
    math::Pose shank_left_pose = shank_left->GetWorldPose();
    // target orientation for the hip
    Q["shank_left"] = math::Quaternion(shank_left_pose.rot.GetRoll(), params[theta_knee+LEG::LEFT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["shank_left"] = shank_left_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["shank_left"] = shank_left->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["shank_left"] = shank_left->GetWorldAngularVel();

    // shank right
    physics::LinkPtr shank_right = parent_model->GetLink("shank_right");
    math::Pose shank_right_pose = shank_right->GetWorldPose();
    // target orientation for the hip
    Q["shank_right"] = math::Quaternion(shank_right_pose.rot.GetRoll(), params[theta_knee+LEG::RIGHT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["shank_right"] = shank_right_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["shank_right"] = shank_right->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["shank_right"] = shank_right->GetWorldAngularVel();

    // foot left
    physics::LinkPtr foot_left = parent_model->GetLink("foot_left");
    math::Pose foot_left_pose = foot_left->GetWorldPose();
    // target orientation for the hip
    Q["foot_left"] = math::Quaternion(foot_left_pose.rot.GetRoll(), params[theta_ankle+LEG::LEFT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["foot_left"] = foot_left_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["foot_left"] = foot_left->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["foot_left"] = foot_left->GetWorldAngularVel();

    // foot right
    physics::LinkPtr foot_right = parent_model->GetLink("foot_right");
    math::Pose foot_right_pose = foot_left->GetWorldPose();
    // target orientation for the hip
    Q["foot_right"] = math::Quaternion(foot_right_pose.rot.GetRoll(), params[theta_ankle+LEG::RIGHT], psi_heading);
    // there is no target position for the hip, so we set it to the current position
    P["foot_right"] = foot_right_pose.pos;
    // target velocity is the forward velocity into desired heading direction
    v["foot_right"] = foot_right->GetWorldCoGLinearVel();
    // target angular velocity is zero
    omega["foot_right"] = foot_right->GetWorldAngularVel();
}

void WalkController::updateMuscleForces() {
    // calculate target force and torque
    for (auto link_name:link_names) {
        physics::LinkPtr link = parent_model->GetLink(link_name);
        F[link_name] = params[k_P] * (P[link_name] - link->GetWorldPose().pos)
                       + params[k_V] * (v[link_name] - link->GetWorldCoGLinearVel());
        math::Quaternion q = Q[link_name] * link->GetWorldPose().rot.GetInverse();
        math::Vector3 v(q.x, q.y, q.z);
        double norm = v.GetLength();
        math::Vector3 exponent = exp(q.w) * (cos(norm) * math::Vector3::One + v.Normalize() * sin(norm));
        T[link_name] = params[k_Q] * (Q[link_name].GetAsMatrix3() * exponent)
                       + params[k_omega] * (omega[link_name] - link->GetWorldAngularVel());

        // iterate over all joints of link
        physics::Joint_V joints = link->GetChildJoints();
        for (auto joint:joints) {
            // calculate the target Force of each muscle spanning this joint
            for (uint muscle:muscles_spanning_joint[joint->GetName()]) {
                math::Vector3 momentArm = sim_muscles[muscle]->momentArm;
                double length = momentArm.GetLength();
                math::Vector3 momentArm_normalized = momentArm.Normalize();
                tau[sim_muscles[muscle]->name] =
                        momentArm_normalized.Cross(link->GetWorldPose().pos - joint->GetWorldPose().pos)
                                .Dot(F[link_name]) + momentArm_normalized.Dot(T[link_name]);
                if(length>0.0)
                    F_tilde[sim_muscles[muscle]->name] = tau[sim_muscles[muscle]->name] / momentArm.GetLength();
                else
                    F_tilde[sim_muscles[muscle]->name] = 0.0;
            }
        }
    }
    ROS_INFO_STREAM("\nF_tilde: " << F_tilde["motor0"] << "\ntau: " << tau["motor0"] << "\nF: " <<
             F["motor0"] << "\nT: " <<  T["motor0"]);
}

void WalkController::updateMuscleActivities(){
    // queue the activities for time delayed feedbacks
    for(uint muscle=0; muscle<sim_muscles.size(); muscle++) {
        activity[sim_muscles[muscle]->name].push_back(F_tilde[sim_muscles[muscle]->name]);
    }
    // update the feedbacks. this depends on the current state of each leg
    for(uint leg=0; leg<=2; leg++){
        switch(leg_state[leg]){
            case Stance: {
                // during stance the hip muscles stabilize and rotate the hip towards its target orientation
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    feedback[sim_muscles[muscle]->name] =
                            activity[sim_muscles[muscle]->name].front() + params[c_stance_lift];
                }
                // the rest of the leg has no target position or orientations, instead we are using positive
                // force feedback for the extensors to achieve joint compliance
                sprintf(joint, "knee_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_stance_lift];
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_stance_lift];
                }
                break;
            }
            case Lift_off: {
                // during lift-off all muscles attached to the hip are fed a constant
                // excitation of high magnitude, to initiate a leg swing
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] = params[c_hip_lift] + params[c_stance_lift];
                    if (sim_muscles[muscle]->muscle_type == FLEXOR)
                        feedback[sim_muscles[muscle]->name] = -params[c_hip_lift] + params[c_stance_lift];
                }
                // the rest of the leg has no target position or orientations, instead we are using positive
                // force feedback for the extensors to achieve joint compliance
                sprintf(joint, "knee_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_stance_lift];
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_stance_lift];
                }
                break;
            }
            case Swing: {
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    feedback[sim_muscles[muscle]->name] =
                            activity[sim_muscles[muscle]->name].front() + params[c_swing_prep];
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_swing_prep];
                }
                break;
            }
            case Stance_Preparation: {
                char joint[20];
                sprintf(joint, "groin_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    feedback[sim_muscles[muscle]->name] =
                            activity[sim_muscles[muscle]->name].front() + params[c_swing_prep];
                }
                sprintf(joint, "knee_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_swing_prep];
                }
                sprintf(joint, "ankle_%s", (leg == LEFT ? "left" : "right"));
                for (uint muscle:muscles_spanning_joint[joint]) {
                    if (sim_muscles[muscle]->muscle_type == EXTENSOR)
                        feedback[sim_muscles[muscle]->name] =
                                params[k_M_Fplus] * activity[sim_muscles[muscle]->name].front()
                                + params[c_swing_prep];
                }
                break;
            }
        }
    }
    // integrate the activity and feedback and pop the first activity
    // set command of each muscle to the integrated activity
    for(uint muscle=0; muscle<sim_muscles.size(); muscle++) {
        a[sim_muscles[muscle]->name] += 100.0*gazebo_max_step_size*
                        (feedback[sim_muscles[muscle]->name] - activity[sim_muscles[muscle]->name].front());
        activity[sim_muscles[muscle]->name].pop_front();
        sim_muscles[muscle]->cmd = a[sim_muscles[muscle]->name];
    }
    ROS_INFO("a: %lf, activity(%lu): %lf, feedback %lf, Fmax %lf", a["motor0"], activity["motor0"].size(),
             activity["motor0"].front(), feedback["motor0"], params[F_max]);
}

void WalkController::updateEnergies(){
    E_speed += fabs(1.0-v_base/v_forward);
    if(E_speed < H_speed)
        E_speed = 0;
    E_speed_int = w_speed * E_speed;
    math::Quaternion q = Q["hip"] * parent_model->GetLink("hip")->GetWorldPose().rot.GetInverse();
    math::Vector3 v(q.x, q.y, q.z);
    double norm = v.GetLength();
    math::Vector3 exponent = exp(q.w) * (cos(norm) * math::Vector3::One + v.Normalize() * sin(norm));
    E_headori += (Q["hip"].GetAsMatrix3() * exponent ).GetLength();
    if(E_headori < H_headori)
        E_headori = 0;
    E_headori_int = w_headori * E_headori;
    for(uint i=0; i<sim_muscles.size(); i++){
        E_effort += sim_muscles[i]->cmd;
    }
    if(E_effort < H_effort)
        E_effort = 0;
    E_effort_int = w_effort * E_effort;

}

bool WalkController::checkAbort(){
    if(center_of_mass[POSITION].z<0.1*initial_center_of_mass_height.z) {
        roboy_simulation::Abortion msg;
        msg.roboyID = roboyID;
        msg.reason = COMheight;
        abort_pub.publish(msg);
        ROS_WARN_THROTTLE(1.0,"center of mass below threshold, aborting");
        return true;
    }
    if(radiansToDegrees(fabs(hip_CS->rot.GetYaw())-fabs(psi_heading)) > 45.0f) {
        roboy_simulation::Abortion msg;
        msg.roboyID = roboyID;
        msg.reason = headingDeviation;
        abort_pub.publish(msg);
        ROS_WARN_THROTTLE(1.0,"deviation from target heading above threshold, aborting");
        return true;
    }
    for(auto link_name:link_names){
        physics::Collision_V collisions = parent_model->GetLink(link_name)->GetCollisions();
        for(auto collision:collisions){
            physics::LinkPtr link = collision->GetLink();
            if(link->GetName().find("foot")!=string::npos || link->GetName().find("hip")!=string::npos)// collsions for
                // the feed are ok or hip?!
                continue;
            else {
                roboy_simulation::Abortion msg;
                msg.roboyID = roboyID;
                msg.reason = selfCollision;
                abort_pub.publish(msg);
//                ROS_WARN_THROTTLE(1.0, "self collision detected with %s, aborting", link->GetName().c_str());
                return true;
            }
        }
    }
    return false;
}

void WalkController::eStopCB(const std_msgs::BoolConstPtr &msg) {
    e_stop_active = msg->data;
}

bool WalkController::parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo> &myoMuscles) {
    // initialize TiXmlDocument doc with a string
    TiXmlDocument doc;
    if (!doc.Parse(sdf.c_str()) && doc.Error()) {
        ROS_ERROR("Can't parse MyoMuscles. Invalid robot description.");
        return false;
    }

    // Find joints in transmission tags
    TiXmlElement *root = doc.RootElement();

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *myoMuscle_it = NULL;
    for (myoMuscle_it = root->FirstChildElement("myoMuscle"); myoMuscle_it;
         myoMuscle_it = myoMuscle_it->NextSiblingElement("myoMuscle")) {
        roboy_simulation::MyoMuscleInfo myoMuscle;
        if (myoMuscle_it->Attribute("name")) {
            myoMuscle.name = myoMuscle_it->Attribute("name");
            // myoMuscle joint acting on
            TiXmlElement *link_child_it = NULL;
            for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
                 link_child_it = link_child_it->NextSiblingElement("link")) {
                string linkname = link_child_it->Attribute("name");
                physics::LinkPtr link = parent_model->GetLink(linkname);
                if ((!linkname.empty()) && link) {
                    TiXmlElement *viaPoint_child_it = NULL;
                    for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
                         viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
                        roboy_simulation::ViaPointInfo vp;
                        vp.link = link;
                        float x, y, z;
                        if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
                            return false;
                        }
                        vp.point = math::Vector3(x,y,z);
                        if (viaPoint_child_it->Attribute("type")){
                            string type = viaPoint_child_it->Attribute("type");
                            if (type == "FIXPOINT") {
                                vp.type = roboy_simulation::IViaPoints::FIXPOINT;
                            } else if (type == "SPHERICAL" || type == "CYLINDRICAL") {
                                if (viaPoint_child_it->QueryDoubleAttribute("radius", &vp.radius) != TIXML_SUCCESS){
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading radius");
                                    return false;
                                }
                                if (viaPoint_child_it->QueryIntAttribute("state", &vp.state) != TIXML_SUCCESS){
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading state");
                                    return false;
                                }
                                if (viaPoint_child_it->QueryIntAttribute("revCounter", &vp.revCounter) != TIXML_SUCCESS){
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading revCounter");
                                    return false;
                                }
                                if (type == "SPHERICAL") {
                                    vp.type = roboy_simulation::IViaPoints::SPHERICAL;
                                } else {
                                    vp.type = roboy_simulation::IViaPoints::CYLINDRICAL;
                                }
                            } else if (type == "MESH") {
                                // TODO
                            } else {
                                ROS_ERROR_STREAM_NAMED("parser", "unknown type of via point: " + type);
                                return false;
                            }
                        } else {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading type");
                            return false;
                        }
                        myoMuscle.viaPoints.push_back(vp);
                    }
                    if (myoMuscle.viaPoints.empty()) {
                        ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                << myoMuscle.name << "' link element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                            << myoMuscle.name << "'.");
                    continue;
                }
            }
            ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str() );

            //check if wrapping surfaces are enclosed by fixpoints
            for(int i = 0; i < myoMuscle.viaPoints.size(); i++){
                if(i == 0 && myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT){
                    ROS_ERROR_STREAM_NAMED("parser", "muscle insertion has to be a fix point");
                    return false;
                }
                if(i == myoMuscle.viaPoints.size()-1 && myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT){
                    ROS_ERROR_STREAM_NAMED("parser", "muscle fixation has to be a fix point");
                    return false;
                }
                if(myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT){
                    if(myoMuscle.viaPoints[i-1].type != roboy_simulation::IViaPoints::FIXPOINT
                       || myoMuscle.viaPoints[i+1].type != roboy_simulation::IViaPoints::FIXPOINT){
                        ROS_ERROR_STREAM_NAMED("parser", "non-FIXPOINT via-points have to be enclosed by two FIXPOINT via-points");
                        return false;
                    }
                }
            }

            TiXmlElement *spans_joint_child_it = NULL;
            for (spans_joint_child_it = myoMuscle_it->FirstChildElement("spanningJoint"); spans_joint_child_it;
                 spans_joint_child_it = spans_joint_child_it->NextSiblingElement("spanningJoint")) {
                string jointname = spans_joint_child_it->Attribute("name");
                if (!jointname.empty()) {
                    myoMuscle.spanningJoint = parent_model->GetJoint(jointname);
                    if(strcmp(spans_joint_child_it->GetText(),"extensor")==0)
                        myoMuscle.muscle_type = EXTENSOR;
                    else if(strcmp(spans_joint_child_it->GetText(),"flexor")==0)
                        myoMuscle.muscle_type = FLEXOR;
                    else if(strcmp(spans_joint_child_it->GetText(),"stabilizer")==0)
                        myoMuscle.muscle_type = STABILIZER;
                    else
                        ROS_WARN_STREAM_NAMED("parser", "muscle type not defined for " << myoMuscle.name << "'.");
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No spanningJoint name attribute specified for myoMuscle'"
                            << myoMuscle.name << "'.");
                    continue;
                }
            }

            TiXmlElement *motor_child = myoMuscle_it->FirstChildElement("motor");
            if (motor_child) {
                // bemf_constant
                TiXmlElement *bemf_constant_child = motor_child->FirstChildElement("bemf_constant");
                if (bemf_constant_child) {
                    if (sscanf(bemf_constant_child->GetText(), "%lf", &myoMuscle.motor.BEMFConst) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading bemf_constant constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No bemf_constant element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // torque_constant
                TiXmlElement *torque_constant_child = motor_child->FirstChildElement("torque_constant");
                if (torque_constant_child) {
                    if (sscanf(torque_constant_child->GetText(), "%lf", &myoMuscle.motor.torqueConst) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading torque_constant constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No torque_constant element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // inductance
                TiXmlElement *inductance_child = motor_child->FirstChildElement("inductance");
                if (inductance_child) {
                    if (sscanf(inductance_child->GetText(), "%lf", &myoMuscle.motor.inductance) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading inductance constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No inductance element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // resistance
                TiXmlElement *resistance_child = motor_child->FirstChildElement("resistance");
                if (resistance_child) {
                    if (sscanf(resistance_child->GetText(), "%lf", &myoMuscle.motor.resistance) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading resistance constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No resistance element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // inertiaMoment
                TiXmlElement *inertiaMoment_child = motor_child->FirstChildElement("inertiaMoment");
                if (inertiaMoment_child) {
                    if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.motor.inertiaMoment) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser", "No motor element found in myoMuscle '" << myoMuscle.name <<
                                                                                         "', using default parameters");
            }

            TiXmlElement *gear_child = myoMuscle_it->FirstChildElement("gear");
            if (gear_child) {
                // ratio
                TiXmlElement *ratio_child = gear_child->FirstChildElement("ratio");
                if (ratio_child) {
                    if (sscanf(ratio_child->GetText(), "%lf", &myoMuscle.gear.ratio) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading ratio constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No ratio element found in myoMuscle '"
                            << myoMuscle.name << "' gear element.");
                    return false;
                }
                // ratio
                TiXmlElement *efficiency_child = gear_child->FirstChildElement("efficiency");
                if (efficiency_child) {
                    if (sscanf(efficiency_child->GetText(), "%lf", &myoMuscle.gear.efficiency) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading efficiency constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No efficiency element found in myoMuscle '"
                            << myoMuscle.name << "' gear element.");
                    return false;
                }
                // inertiaMoment
                TiXmlElement *inertiaMoment_child = gear_child->FirstChildElement("inertiaMoment");
                if (inertiaMoment_child) {
                    if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.gear.inertiaMoment) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
                            << myoMuscle.name << "' gear element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser", "No gear element found in myoMuscle '" << myoMuscle.name <<
                                                                                        "', using default parameters");
            }

            TiXmlElement *spindle_child = myoMuscle_it->FirstChildElement("spindle");
            if (spindle_child) {
                // radius
                TiXmlElement *radius_child = spindle_child->FirstChildElement("radius");
                if (radius_child) {
                    if (sscanf(radius_child->GetText(), "%lf", &myoMuscle.spindle.radius) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No radius element found in myoMuscle '"
                            << myoMuscle.name << "' spindle element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser",
                                       "No spindle element found in myoMuscle '" << myoMuscle.name <<
                                                                                 "', using default parameters");
            }

            TiXmlElement *SEE_child = myoMuscle_it->FirstChildElement("SEE");
            if (SEE_child) {
                // stiffness
                TiXmlElement *stiffness_child = SEE_child->FirstChildElement("stiffness");
                if (stiffness_child) {
                    if (sscanf(stiffness_child->GetText(), "%lf", &myoMuscle.see.stiffness) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No stiffness element found in myoMuscle '"
                            << myoMuscle.name << "' SEE element.");
                    return false;
                }
                // length
                TiXmlElement *length_child = SEE_child->FirstChildElement("length");
                if (length_child) {
                    if (sscanf(length_child->GetText(), "%lf", &myoMuscle.see.length) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading length constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No length element found in myoMuscle '"
                            << myoMuscle.name << "' SEE element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser", "No SEE element found in myoMuscle '" << myoMuscle.name <<
                                                                                       "', using default parameters");
            }

        } else {
            ROS_ERROR_STREAM_NAMED("parser",
                                   "No name attribute specified for myoMuscle, please name the muscle in sdf file");
            return false;
        }
        myoMuscles.push_back(myoMuscle);
    }
    TiXmlElement *foot_sole_left = NULL;
    foot_sole_left = root->FirstChildElement("foot_sole_left");
    if (foot_sole_left) {
        float x, y, z;
        if (sscanf(foot_sole_left->GetText(), "%lf %lf %lf", &foot_sole[LEG::LEFT].x, &foot_sole[LEG::LEFT].y,
                   &foot_sole[LEG::LEFT].z) != 3) {
            ROS_ERROR_STREAM_NAMED("parser", "error reading [foot_sole_left] (x y z)");
            return false;
        }
    } else {
        ROS_WARN_STREAM_NAMED("parser", "No foot_sole_left element found, using " << foot_sole[LEG::LEFT]);
    }
    TiXmlElement *foot_sole_right = NULL;
    foot_sole_right = root->FirstChildElement("foot_sole_right");
    if (foot_sole_right) {
        float x, y, z;
        if (sscanf(foot_sole_right->GetText(), "%lf %lf %lf", &foot_sole[LEG::RIGHT].x, &foot_sole[LEG::RIGHT].y,
                   &foot_sole[LEG::RIGHT].z) != 3) {
            ROS_ERROR_STREAM_NAMED("parser", "error reading [foot_sole_right] (x y z)");
            return false;
        }
    } else {
        ROS_WARN_STREAM_NAMED("parser", "No foot_sole_right element found, using " << foot_sole[LEG::RIGHT]);
    }

    ROS_INFO_STREAM("foot_soles: " << foot_sole[LEG::LEFT] << " / " << foot_sole[LEG::RIGHT]);

    return true;
}

void WalkController::publishID(){
    std_msgs::Int32 msg;
    msg.data = roboyID;
    roboyID_pub.publish(msg);
}

void WalkController::motorControl(const roboy_simulation::MotorControl::ConstPtr &msg){
    // only react to messages for me
    if(msg->roboyID == roboyID) {
        // switch to manual control
        control = false;
        // update commanded motor voltages
        if(msg->voltage.size() == sim_muscles.size()) {
            for (uint i = 0; i < sim_muscles.size(); i++) {
                sim_muscles[i]->cmd = msg->voltage[i];
            }
        }
    }
}

bool WalkController::updateControllerParameters(roboy_simulation::UpdateControllerParameters::Request  &req,
                                                roboy_simulation::UpdateControllerParameters::Response &res){
    messageTocontrollerParameters(req.params, params);
    return true;
}

bool WalkController::energiesService(roboy_simulation::Energies::Request  &req,
                     roboy_simulation::Energies::Response &res){
    res.E_speed = E_speed_int;
    res.E_headvel = E_headvel_int;
    res.E_headori = E_headori_int;
    res.E_slide = E_slide_int;
    res.E_effort = E_effort_int;
    return true;
}
//Basic PID joint control system described below...
void WalkController::controlJoints(){
  roboy_simulation::Input input_msg;
  input_msg.roboyID = roboyID;

  if(firstLoop) {
    currentTime = parent_model->GetWorld()->GetSimTime();
    firstLoop = false;
    return;
  }
  else {
    previousTime = currentTime;
    currentTime = parent_model->GetWorld()->GetSimTime();
    deltaTime = currentTime - previousTime;

    bool IsPositive = false;
    for (auto joint_name : joint_names) {
        physics::JointPtr thisJoint = parent_model->GetJoint(joint_name);
        publishJoints(thisJoint);
        double currentAngle = thisJoint->GetAngle(0).Radian();
        double deltaAngle = currentAngle -desiredAngles[joint_name];
        jointPIDs[joint_name].Update(deltaAngle, deltaTime);
        double inputVoltage = jointPIDs[joint_name].GetCmd();
        if (inputVoltage>0) {IsPositive = true;}
        double effectiveVoltage = abs (inputVoltage);


        if(joint_name == "neck") {
          // No motors existing...
        }
        if(joint_name == "shoulder_left") {
          // No motors existing...
        }
        if(joint_name == "shoulder_right") {
          // No motors existing...
        }
        if(joint_name == "elbow_left") {
          // No motors existing...
        }
        if(joint_name == "elbow_right") {
          // No motors existing...
        }
        if(joint_name == "spine") {
          // No motors existing...
        }
        if(joint_name == "groin_left") {
          if(IsPositive){
            sim_muscles[0]->cmd = effectiveVoltage;
            sim_muscles[2]->cmd = 0;
          }
          else {
            sim_muscles[0]->cmd = 0;
            sim_muscles[2]->cmd = effectiveVoltage;
          }
        }
        if(joint_name == "groin_right") {
          if(IsPositive){
            sim_muscles[8]->cmd = 0;
            sim_muscles[10]->cmd = effectiveVoltage;
          }
          else {
            sim_muscles[8]->cmd = effectiveVoltage;
            sim_muscles[10]->cmd = 0;
          }
        }
        if(joint_name == "knee_left") {
          if(IsPositive){
            sim_muscles[4]->cmd = 0;
            sim_muscles[5]->cmd = effectiveVoltage;
          }
          else {
            sim_muscles[4]->cmd = effectiveVoltage;
            sim_muscles[5]->cmd = 0;
          }
        }
        if(joint_name == "knee_right") {
          if(IsPositive){
            sim_muscles[12]->cmd = 0;
            sim_muscles[13]->cmd = effectiveVoltage;
          }
          else {
            sim_muscles[12]->cmd = effectiveVoltage;
            sim_muscles[13]->cmd = 0;
          }
        }
        if(joint_name == "ankle_left") {
          if(IsPositive){
            sim_muscles[6]->cmd = 0;
            sim_muscles[7]->cmd = effectiveVoltage;
          }
          else {
            sim_muscles[6]->cmd = effectiveVoltage;
            sim_muscles[7]->cmd = 0;
          }
        }
        if(joint_name == "ankle_right") {
          if(IsPositive){
            sim_muscles[14]->cmd = effectiveVoltage;
            sim_muscles[15]->cmd = 0;
          }
          else {
            sim_muscles[14]->cmd = 0;
            sim_muscles[15]->cmd = effectiveVoltage;
          }
        }

        // input voltages for motors are being published here...
        input_msg.name = joint_name;
        input_msg.inputVoltage = inputVoltage;
        input_pub.publish(input_msg);
    }
}

}


void WalkController::publishJoints(const physics::JointPtr thisJoint){
    roboy_simulation::Joint joint_msg;
    joint_msg.roboyID = roboyID;
    joint_msg.name = thisJoint->GetName();
    joint_msg.radian = thisJoint->GetAngle(0).Radian();
    joint_pub.publish(joint_msg);
    return;
}

void WalkController::publishCOMmsg () {
    roboy_simulation::COM COM_msg;
    COM_msg.roboyID = roboyID;
    COM_msg.Position.x = center_of_mass[POSITION].x;
    COM_msg.Position.y = center_of_mass[POSITION].y;
    COM_msg.Position.z = center_of_mass[POSITION].z;
    COM_msg.Velocity.x = center_of_mass[VELOCITY].x;
    COM_msg.Velocity.y = center_of_mass[VELOCITY].y;
    COM_msg.Velocity.z = center_of_mass[VELOCITY].z;
    COM_pub.publish(COM_msg);

    SimulationControl &simcontrol = SimulationControl::getInstance();
    simcontrol.writeRosbagIfRecording("/roboy/COM", COM_msg);
}

math::Vector3 WalkController::getFilteredLinearAcceleration(const physics::LinkPtr link)
{
    math::Vector3 lin_accel_world = link->GetWorldLinearAccel();
    if (!filterIMUs) {
        // Filtering disabled
        return lin_accel_world;
    }
    string link_name = link->GetName();

    vector<math::Vector3> &window = acceleration_windows[link_name];
    if (window.size() != ACCEL_WIN_SIZE) {
        // This should never happen, but if it happens, give a warning
        ROS_WARN_STREAM(link_name << ": Acceleration window size doesn't match ACCEL_WIN_SIZE!");
    }

    // Remove the oldest measurement from the time window of earlier
    // acceleration measurements.
    for (uint i = 0; i < ACCEL_WIN_SIZE-1; i++) {
        window[i] = window[i+1];
    }

    // Add the new measurement to the time window
    window[ACCEL_WIN_SIZE-1] = lin_accel_world;

    // Calculate the low-pass filtered vector
    math::Vector3 filtered_lin_accel;
    for (uint i = 0; i < ACCEL_WIN_SIZE; i++) {
        filtered_lin_accel += window[i] * gaussian_kernel[i];
    }

    return filtered_lin_accel;
}

GZ_REGISTER_MODEL_PLUGIN(WalkController)
