#include "roboy_simulation/balancing_plugin.hpp"

BalancingPlugin::BalancingPlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    // Layout for reset, play, pause, slow motion
    QGroupBox *simcontrolGroupBox = new QGroupBox(tr("Simulation control"), this);
    QHBoxLayout *simcontrol = new QHBoxLayout();

    QPushButton *button = new QPushButton(tr("Reset"));
    connect(button, SIGNAL(clicked()), this, SLOT(resetWorld()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("Play"));
    connect(button, SIGNAL(clicked()), this, SLOT(play()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("Pause"));
    connect(button, SIGNAL(clicked()), this, SLOT(pause()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("Slow motion"));
    connect(button, SIGNAL(clicked()), this, SLOT(slowMotion()));
    simcontrol->addWidget(button);

    simcontrolGroupBox->setLayout(simcontrol);
    frameLayout->addWidget(simcontrolGroupBox);

    // Layouts for visualization options
    QGroupBox *optionsGroupBox = new QGroupBox(tr("Visualizations"), this);
    QHBoxLayout *options = new QHBoxLayout();
    QVBoxLayout *options0 = new QVBoxLayout();
    QVBoxLayout *options1 = new QVBoxLayout();

    QCheckBox *visualizeMesh = new QCheckBox(tr("Meshes"));
    visualizeMesh->setObjectName("visualizeMesh");
    connect(visualizeMesh, SIGNAL(clicked()), this, SLOT(showMesh()));
    options0->addWidget(visualizeMesh);

    QCheckBox *visualizeTendon = new QCheckBox(tr("Tendons"));
    visualizeTendon->setObjectName("visualizeTendon");
    connect(visualizeTendon, SIGNAL(clicked()), this, SLOT(showTendon()));
    options0->addWidget(visualizeTendon);

    QCheckBox *visualizeCOM = new QCheckBox(tr("COM"));
    visualizeCOM->setObjectName("visualizeCOM");
    connect(visualizeCOM, SIGNAL(clicked()), this, SLOT(showCOM()));
    options0->addWidget(visualizeCOM);

    QCheckBox *visualizeForce = new QCheckBox(tr("Forces"));
    visualizeForce->setObjectName("visualizeForce");
    connect(visualizeForce, SIGNAL(clicked()), this, SLOT(showForce()));
    options0->addWidget(visualizeForce);

    QCheckBox *visualizeForceTorqueSensors = new QCheckBox(tr("Force-torque sensors"));
    visualizeForceTorqueSensors->setObjectName("visualizeForceTorqueSensors");
    connect(visualizeForceTorqueSensors, SIGNAL(clicked()), this, SLOT(showForceTorqueSensors()));
    options1->addWidget(visualizeForceTorqueSensors);

    QCheckBox *visualizeIMUs = new QCheckBox(tr("IMU sensors"));
    visualizeIMUs->setObjectName("visualizeIMUs");
    connect(visualizeIMUs, SIGNAL(clicked()), this, SLOT(showIMUs()));
    options1->addWidget(visualizeIMUs);

    QCheckBox *visualizeEstimatedCOM = new QCheckBox(tr("Estimated COM"));
    visualizeEstimatedCOM->setObjectName("visualizeEstimatedCOM");
    connect(visualizeEstimatedCOM, SIGNAL(clicked()), this, SLOT(showEstimatedCOM()));
    options1->addWidget(visualizeEstimatedCOM);

    options->addLayout(options0);
    options->addLayout(options1);
    optionsGroupBox->setLayout(options);
    frameLayout->addWidget(optionsGroupBox);

    QGroupBox *signalOptionsGroupBox = new QGroupBox(tr("Signal processing"), this);
    QHBoxLayout *signalOptions = new QHBoxLayout();
    QVBoxLayout *signalOptions0 = new QVBoxLayout();

    QCheckBox *IMUFiltering = new QCheckBox(tr("Filter IMU data"));
    IMUFiltering->setObjectName("IMUFiltering");
    connect(IMUFiltering, SIGNAL(clicked()), this, SLOT(toggleIMUFiltering()));
    signalOptions0->addWidget(IMUFiltering);

    signalOptions->addLayout(signalOptions0);
    signalOptionsGroupBox->setLayout(signalOptions);
    frameLayout->addWidget(signalOptionsGroupBox);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    // initialize ros

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "RvizBalancingPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;

    spinner = new ros::AsyncSpinner(1);

    roboy_visualization_control_pub = nh->advertise<roboy_simulation::VisualizationControl>("/roboy/visualization_control", 1);
    reset_world_srv = nh->serviceClient<std_srvs::Trigger>("/roboy/reset_world");
    sim_control_pub = nh->advertise<std_msgs::Int32>("/roboy/sim_control", 1);
    motor_control_pub = nh->advertise<roboy_simulation::MotorControl>("/roboy/motor_control", 100);
}

BalancingPlugin::~BalancingPlugin(){
    delete nh;
    delete spinner;
}

void BalancingPlugin::save(rviz::Config config) const {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeTendon");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeCOM");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeForce");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    config.mapSetValue(w->objectName(), w->isChecked());
    w = this->findChild<QCheckBox*>("visualizeIMUs");
    config.mapSetValue(w->objectName(), w->isChecked());
    rviz::Panel::save(config);
}

void BalancingPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    bool checked = false;
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeTendon");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeCOM");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeForce");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("visualizeIMUs");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
    w = this->findChild<QCheckBox*>("IMUFiltering");
    config.mapGetBool(w->objectName(), &checked);
    w->setChecked(checked);
}

void BalancingPlugin::showCOM() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeCOM");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = COM;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showEstimatedCOM() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeEstimatedCOM");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = EstimatedCOM;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showForce() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeForce");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = Forces;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showTendon() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeTendon");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = Tendon;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showMesh() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = Mesh;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showForceTorqueSensors() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = ForceTorqueSensors;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showIMUs() {
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeIMUs");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = IMUs;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::toggleIMUFiltering() {
    QCheckBox* w = this->findChild<QCheckBox*>("IMUFiltering");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = IMUFiltering;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::resetWorld() {
    std_srvs::Trigger srv;
    reset_world_srv.call(srv);
}

void BalancingPlugin::play() {
    std_msgs::Int32 msg;
    msg.data = Play;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::pause() {
    std_msgs::Int32 msg;
    msg.data = Pause;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::slowMotion() {
    std_msgs::Int32 msg;
    msg.data = Slow_Motion;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::refresh() {
    showCOM();
    showEstimatedCOM();
    showForce();
    showForceTorqueSensors();
    showIMUs();
    showMesh();
    showTendon();
    toggleIMUFiltering();
}

PLUGINLIB_EXPORT_CLASS(BalancingPlugin, rviz::Panel)
