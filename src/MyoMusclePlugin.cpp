#include "roboy_simulation/MyoMusclePlugin.hpp"

int MyoMusclePlugin::roboyID_generator = 0;

MyoMusclePlugin::MyoMusclePlugin() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "MyoMusclePlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner.reset(new ros::AsyncSpinner(1));
    spinner->start();

    roboyID_pub = nh->advertise<std_msgs::Int32>("/roboy/id", 1);
    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &MyoMusclePlugin::MotorCommand, this);
    motorStatus_pub = nh->advertise<roboy_communication_middleware::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    roboyID = roboyID_generator++;
    ID = roboyID;
}

MyoMusclePlugin::~MyoMusclePlugin() {
    motor_status_publishing = false;
    if(motor_status_publisher!=nullptr)
        motor_status_publisher->join();
}

void MyoMusclePlugin::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
    ROS_INFO("Loading MyoMusclePlugin plugin");
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
    }

    physics::PhysicsEnginePtr physics_engine = parent_model->GetWorld()->GetPhysicsEngine();

    // Get the Gazebo solver type
    std::string solver_type = boost::any_cast<std::string>(physics_engine->GetParam("solver_type"));

    // Give the global CFM (Constraint Force Mixing) a positive value (default 0) making the model a bit less stiff to avoid 			jittering
    // Positive CFM allows links to overlap each other so the collisions aren't that hard
    physics_engine->SetParam("cfm", 0.00);

    ROS_INFO_STREAM("Simulation max_step_size: " << gazebo_max_step_size << " s");

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
    control_period = gazebo_period;

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
            //a[sim_muscles[muscle]->name] = 0.0;
        }
        catch (pluginlib::PluginlibException &ex) {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&MyoMusclePlugin::Update, this));

    motor_status_publisher.reset(new boost::thread(&MyoMusclePlugin::MotorStatusPublisher,this));
    motor_status_publisher->detach();

    ROS_INFO("MyoMusclePlugin ready");
}

void MyoMusclePlugin::Update() {
    // Get the simulation time and period
    gz_time_now = parent_model->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;
    last_update_sim_time_ros = sim_time_ros;
    if(!firstLoop) {
        // get modelpose and update mucles
        readSim(sim_time_ros, sim_period);
        // Update the gazebo model with the result of the controller computation
        writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);
    }else{
        firstLoop = false;
    }
    last_write_sim_time_ros = sim_time_ros;
    message_counter = 1000;

    publishID();
    publishForce(&sim_muscles);
    publishTendon(&sim_muscles);
    // Parent_moddel->GetName comes from launch file
    publishModel(parent_model->GetName(), parent_model->GetLink(link_names[0]), false);
}

void MyoMusclePlugin::readSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("read simulation");
    // update muscle plugins
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        for (int i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++) {
            math::Pose linkPose = sim_muscles[muscle]->viaPoints[i]->link->GetWorldPose();
            sim_muscles[muscle]->viaPoints[i]->linkPosition = linkPose.pos;
            sim_muscles[muscle]->viaPoints[i]->linkRotation = linkPose.rot;
        }
        sim_muscles[muscle]->Update(time, period);
    }
}

void MyoMusclePlugin::writeSim(ros::Time time, ros::Duration period) {
    ROS_DEBUG("write simulation");
    // apply the calculated forces
    for (uint muscle = 0; muscle < sim_muscles.size(); muscle++) {
        for (int i = 0; i < sim_muscles[muscle]->viaPoints.size(); i++) {
            std::shared_ptr<roboy_simulation::IViaPoints> vp = sim_muscles[muscle]->viaPoints[i];
            if (vp->prevForcePoint.IsFinite() && vp->nextForcePoint.IsFinite()) {
                vp->link->AddForceAtWorldPosition(vp->prevForce, vp->prevForcePoint);
                vp->link->AddForceAtWorldPosition(vp->nextForce, vp->nextForcePoint);
            }
        }
    }
}

void MyoMusclePlugin::Reset() {
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros = ros::Time();
    last_write_sim_time_ros = ros::Time();
    //reset acceleration of links and the actuator simulation.
}

bool MyoMusclePlugin::parseMyoMuscleSDF(const string &sdf, vector<roboy_simulation::MyoMuscleInfo> &myoMuscles) {
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
                        vp.point = math::Vector3(x, y, z);
                        if (viaPoint_child_it->Attribute("type")) {
                            string type = viaPoint_child_it->Attribute("type");
                            if (type == "FIXPOINT") {
                                vp.type = roboy_simulation::IViaPoints::FIXPOINT;
                            } else if (type == "SPHERICAL" || type == "CYLINDRICAL") {
                                if (viaPoint_child_it->QueryDoubleAttribute("radius", &vp.radius) != TIXML_SUCCESS) {
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading radius");
                                    return false;
                                }
                                if (viaPoint_child_it->QueryIntAttribute("state", &vp.state) != TIXML_SUCCESS) {
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading state");
                                    return false;
                                }
                                if (viaPoint_child_it->QueryIntAttribute("revCounter", &vp.revCounter) !=
                                    TIXML_SUCCESS) {
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
            ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str());

            //check if wrapping surfaces are enclosed by fixpoints
            for (int i = 0; i < myoMuscle.viaPoints.size(); i++) {
                if (i == 0 && myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT) {
                    ROS_ERROR_STREAM_NAMED("parser", "muscle insertion has to be a fix point");
                    return false;
                }
                if (i == myoMuscle.viaPoints.size() - 1 &&
                    myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT) {
                    ROS_ERROR_STREAM_NAMED("parser", "muscle fixation has to be a fix point");
                    return false;
                }
                if (myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT) {
                    if (myoMuscle.viaPoints[i - 1].type != roboy_simulation::IViaPoints::FIXPOINT
                        || myoMuscle.viaPoints[i + 1].type != roboy_simulation::IViaPoints::FIXPOINT) {
                        ROS_ERROR_STREAM_NAMED("parser",
                                               "non-FIXPOINT via-points have to be enclosed by two FIXPOINT via-points");
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
                    if (strcmp(spans_joint_child_it->GetText(), "extensor") == 0)
                        myoMuscle.muscle_type = EXTENSOR;
                    else if (strcmp(spans_joint_child_it->GetText(), "flexor") == 0)
                        myoMuscle.muscle_type = FLEXOR;
                    else if (strcmp(spans_joint_child_it->GetText(), "stabilizer") == 0)
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
    return true;
}

void MyoMusclePlugin::publishID() {
    std_msgs::Int32 msg;
    msg.data = roboyID;
    roboyID_pub.publish(msg);
}

void MyoMusclePlugin::MotorCommand(const roboy_communication_middleware::MotorCommand::ConstPtr &msg) {
    // only react to messages for me
    if (msg->id == roboyID) {
        // update pid setvalues
        for (uint i = 0; i < msg->motors.size(); i++) {
            if(msg->motors[i]<sim_muscles.size()) {
                sim_muscles[msg->motors[i]]->pid_control = true;
                sim_muscles[msg->motors[i]]->feedback_type = 2;
                sim_muscles[msg->motors[i]]->cmd = msg->setPoints[i]*0.01; // convert displacement ticks to mm
            }
        }
    }
}

void MyoMusclePlugin::MotorStatusPublisher(){
    ros::Rate rate(100);
    while(motor_status_publishing){
        roboy_communication_middleware::MotorStatus msg;
        for(auto const &muscle:sim_muscles){
            msg.pwmRef.push_back(muscle->cmd);
            msg.position.push_back(muscle->actuator.gear.position);
            msg.velocity.push_back(muscle->actuator.spindle.angVel*M_PI*2.0); // convert 1/s to radians/sec
            msg.displacement.push_back(muscle->see.deltaX/0.01); // convert mm to displacement ticks
            msg.current.push_back(muscle->actuator.motor.current);
        }
        motorStatus_pub.publish(msg);
        rate.sleep();
    }
}

GZ_REGISTER_MODEL_PLUGIN(MyoMusclePlugin)
