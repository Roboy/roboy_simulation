#include "roboy_simulation/balancing_plugin.hpp"

BalancingPlugin::BalancingPlugin(QWidget *parent)
        : rviz::Panel(parent) {
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    // Layout for reset, play, pause, slow motion
    QGroupBox *simcontrolGroupBox = new QGroupBox(tr("Simulation control"), this);
    QVBoxLayout *simulationControls = new QVBoxLayout();
    QHBoxLayout *simcontrol = new QHBoxLayout();
    QHBoxLayout *recordingControl = new QHBoxLayout();

    QPushButton *button = new QPushButton(tr("Reset"));
    connect(button, SIGNAL(clicked()), this, SLOT(resetWorld()));
    simcontrol->addWidget(button);

    // Simulation control buttons
    button = new QPushButton(tr("Play"));
    connect(button, SIGNAL(clicked()), this, SLOT(play()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("Pause"));
    connect(button, SIGNAL(clicked()), this, SLOT(pause()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("Slow motion"));
    connect(button, SIGNAL(clicked()), this, SLOT(slowMotion()));
    simcontrol->addWidget(button);

    // Recording buttons
    startRec = new QPushButton(tr("Start recording"));
    stopRec = new QPushButton(tr("Stop recording"));

    connect(startRec, SIGNAL(clicked()), this, SLOT(startRecording()));
    startRec->setEnabled(true);
    recordingControl->addWidget(startRec);

    connect(stopRec, SIGNAL(clicked()), this, SLOT(stopRecording()));
    stopRec->setEnabled(false);
    recordingControl->addWidget(stopRec);

    QFormLayout *recordingTimes = new QFormLayout;
    startTime = new QTimeEdit(this);
    stopTime = new QTimeEdit(this);
    startTime->setDisplayFormat("HH:mm:ss.zzz");
    stopTime->setDisplayFormat("HH:mm:ss.zzz");

    resetAndRecord = new QPushButton("Reset and record given timespan");
    connect(resetAndRecord, SIGNAL(clicked()), this, SLOT(resetAndStartRecording()));

    recordingTimes->addRow(tr("Start time:"), startTime);
    recordingTimes->addRow(tr("Stop time:"), stopTime);
    recordingTimes->addRow(tr(" "), resetAndRecord);

    simulationControls->addLayout(simcontrol);
    simulationControls->addLayout(recordingControl);
    simulationControls->addLayout(recordingTimes);
    simcontrolGroupBox->setLayout(simulationControls);
    frameLayout->addWidget(simcontrolGroupBox);

    // Layouts for visualization options
    QGroupBox *optionsGroupBox = new QGroupBox(tr("Visualizations"), this);
    QHBoxLayout *options = new QHBoxLayout();
    QVBoxLayout *options0 = new QVBoxLayout();
    QVBoxLayout *options1 = new QVBoxLayout();

    QCheckBox *checkbox = new QCheckBox(tr("Meshes"));
    checkbox->setObjectName(checkbox_names[Mesh]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showMesh()));
    options0->addWidget(checkbox);
    checkboxes[Mesh] = checkbox;

    checkbox = new QCheckBox(tr("Tendons"));
    checkbox->setObjectName(checkbox_names[Tendon]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showTendon()));
    options0->addWidget(checkbox);
    checkboxes[Tendon] = checkbox;

    checkbox = new QCheckBox(tr("COM"));
    checkbox->setObjectName(checkbox_names[COM]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showCOM()));
    options0->addWidget(checkbox);
    checkboxes[COM] = checkbox;

    checkbox = new QCheckBox(tr("Forces"));
    checkbox->setObjectName(checkbox_names[Forces]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showForce()));
    options0->addWidget(checkbox);
    checkboxes[Forces] = checkbox;

    checkbox = new QCheckBox(tr("Force-torque sensors"));
    checkbox->setObjectName(checkbox_names[ForceTorqueSensors]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showForceTorqueSensors()));
    options1->addWidget(checkbox);
    checkboxes[ForceTorqueSensors] = checkbox;

    checkbox = new QCheckBox(tr("IMU sensors"));
    checkbox->setObjectName(checkbox_names[IMUs]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showIMUs()));
    options1->addWidget(checkbox);
    checkboxes[IMUs] = checkbox;

    checkbox = new QCheckBox(tr("Estimated COM"));
    checkbox->setObjectName(checkbox_names[EstimatedCOM]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showEstimatedCOM()));
    options1->addWidget(checkbox);
    checkboxes[EstimatedCOM] = checkbox;

    checkbox = new QCheckBox(tr("Collision model"));
    checkbox->setObjectName(checkbox_names[EstimatedCOM]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(showCollisions()));
    options1->addWidget(checkbox);
    checkboxes[CollisionModel] = checkbox;

    options->addLayout(options0);
    options->addLayout(options1);
    optionsGroupBox->setLayout(options);
    frameLayout->addWidget(optionsGroupBox);

    QGroupBox *signalOptionsGroupBox = new QGroupBox(tr("Signal processing"), this);
    QHBoxLayout *signalOptions = new QHBoxLayout();
    QVBoxLayout *signalOptions0 = new QVBoxLayout();

    checkbox = new QCheckBox(tr("Filter IMU data"));
    checkbox->setObjectName(checkbox_names[IMUFiltering]);
    connect(checkbox, SIGNAL(clicked()), this, SLOT(toggleIMUFiltering()));
    signalOptions0->addWidget(checkbox);
    checkboxes[IMUFiltering] = checkbox;

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
    for (auto checkbox : checkboxes) {
        config.mapSetValue(checkbox->objectName(), checkbox->isChecked());
    }
    rviz::Panel::save(config);
}

void BalancingPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    bool checked = false;
    for (auto checkbox : checkboxes) {
        config.mapGetBool(checkbox->objectName(), &checked);
        checkbox->setChecked(checked);
    }
    refresh();
}

void BalancingPlugin::publishCheckBoxState(VISUALIZATION checkbox) {
    QCheckBox* w = checkboxes[checkbox];
    if (w == nullptr) {
        ROS_WARN_STREAM(checkbox_names[checkbox].toUtf8().constData() << ": No such checkbox found!");
        return;
    }
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = checkbox;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::publishControlMessage(SIMULATIONCONTROL msg) {
    std_msgs::Int32 message;
    message.data = msg;
    sim_control_pub.publish(message);
}

void BalancingPlugin::showCOM() {
    publishCheckBoxState(COM);
}

void BalancingPlugin::showEstimatedCOM() {
    publishCheckBoxState(EstimatedCOM);
}

void BalancingPlugin::showCollisions() {
    publishCheckBoxState(CollisionModel);
}

void BalancingPlugin::showForce() {
    publishCheckBoxState(Forces);
}

void BalancingPlugin::showTendon() {
    publishCheckBoxState(Tendon);
}

void BalancingPlugin::showMesh() {
    publishCheckBoxState(Mesh);
}

void BalancingPlugin::showForceTorqueSensors() {
    publishCheckBoxState(ForceTorqueSensors);
}

void BalancingPlugin::showIMUs() {
    publishCheckBoxState(IMUs);
}

void BalancingPlugin::toggleIMUFiltering() {
    publishCheckBoxState(IMUFiltering);
}

void BalancingPlugin::resetWorld() {
    std_srvs::Trigger srv;
    reset_world_srv.call(srv);
}

void BalancingPlugin::play() {
    publishControlMessage(Play);
}

void BalancingPlugin::pause() {
    publishControlMessage(Pause);
}

void BalancingPlugin::slowMotion() {
    publishControlMessage(Slow_Motion);
}

void BalancingPlugin::startRecording() {
    startRec->setEnabled(false);
    stopRec->setEnabled(true);
    publishControlMessage(StartRecording);
}

void BalancingPlugin::stopRecording() {
    startRec->setEnabled(true);
    stopRec->setEnabled(false);
    publishControlMessage(StopRecording);
}

void BalancingPlugin::resetAndStartRecording() {
    if (startTime->time() >= stopTime->time()) {
        // Invalid negative timespan
        QMessageBox msgBox;
        msgBox.setWindowTitle("Invalid negative timespan!");
        msgBox.setText("The <b>stop time</b> needs to be greater than the <b>start time</b>.");
        msgBox.exec();
    }
    else {
        QTime zero(0, 0, 0);
        int start_time = zero.msecsTo(startTime->time());
        int stop_time = zero.msecsTo(stopTime->time());
        publishControlMessage(ResetAndStartRecording);
    }
}

void BalancingPlugin::refresh() {
    showCOM();
    showEstimatedCOM();
    showForce();
    showForceTorqueSensors();
    showIMUs();
    showMesh();
    showTendon();
    showCollisions();
    toggleIMUFiltering();
}

PLUGINLIB_EXPORT_CLASS(BalancingPlugin, rviz::Panel)
