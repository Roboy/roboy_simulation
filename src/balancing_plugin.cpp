#include "roboy_simulation/balancing_plugin.hpp"

BalancingPlugin::BalancingPlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QHBoxLayout *roboyIdlayout = new QHBoxLayout();
    QLabel *roboyIDlabel= new QLabel(tr("roboy ID:"));
    roboyIdlayout->addWidget(roboyIDlabel);

    QComboBox *roboyID = new QComboBox();
    roboyID->setObjectName("roboyID");
    connect(roboyID, SIGNAL(currentIndexChanged(int)), this, SLOT(changeID(int)));
    roboyIdlayout->addWidget(roboyID);

    QPushButton *refreshbutton = new QPushButton(tr("refresh"));
    connect(refreshbutton, SIGNAL(clicked()), this, SLOT(refresh()));
    roboyIdlayout->addWidget(refreshbutton);

    frameLayout->addLayout(roboyIdlayout);

    QHBoxLayout *simcontrol = new QHBoxLayout();
    QHBoxLayout *options = new QHBoxLayout();
    QVBoxLayout *options0 = new QVBoxLayout();
    QVBoxLayout *options1 = new QVBoxLayout();

    QHBoxLayout *controllerOptions = new QHBoxLayout();

    frameLayout->addLayout(controllerOptions);

    QPushButton *button = new QPushButton(tr("reset"));
    connect(button, SIGNAL(clicked()), this, SLOT(resetWorld()));
    simcontrol->addWidget(button);

    button= new QPushButton(tr("play"));
    connect(button, SIGNAL(clicked()), this, SLOT(play()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("pause"));
    connect(button, SIGNAL(clicked()), this, SLOT(pause()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("slow motion"));
    connect(button, SIGNAL(clicked()), this, SLOT(slowMotion()));
    simcontrol->addWidget(button);

    button = new QPushButton(tr("update interactive marker"));
    connect(button, SIGNAL(clicked()), this, SLOT(updateInteractiveMarker()));
    simcontrol->addWidget(button);

    frameLayout->addLayout(simcontrol);

    QCheckBox *visualizeMesh= new QCheckBox(tr("show mesh"));
    visualizeMesh->setObjectName("visualizeMesh");
    connect(visualizeMesh, SIGNAL(clicked()), this, SLOT(showMesh()));
    options0->addWidget(visualizeMesh);

    QCheckBox *visualizeTendon= new QCheckBox(tr("show tendon"));
    visualizeTendon->setObjectName("visualizeTendon");
    connect(visualizeTendon, SIGNAL(clicked()), this, SLOT(showTendon()));
    options0->addWidget(visualizeTendon);

    QCheckBox *visualizeCOM = new QCheckBox(tr("show COM"));
    visualizeCOM->setObjectName("visualizeCOM");
    connect(visualizeCOM, SIGNAL(clicked()), this, SLOT(showCOM()));
    options0->addWidget(visualizeCOM);

    QCheckBox *visualizeForce = new QCheckBox(tr("show force"));
    visualizeForce->setObjectName("visualizeForce");
    connect(visualizeForce, SIGNAL(clicked()), this, SLOT(showForce()));
    options0->addWidget(visualizeForce);

    QCheckBox *visualizeForceTorqueSensors = new QCheckBox(tr("show force torque sensors"));
    visualizeForceTorqueSensors->setObjectName("visualizeForceTorqueSensors");
    connect(visualizeForceTorqueSensors, SIGNAL(clicked()), this, SLOT(showForceTorqueSensors()));
    options1->addWidget(visualizeForceTorqueSensors);

    QCheckBox *visualizeIMUs = new QCheckBox(tr("show IMU sensors"));
    visualizeIMUs->setObjectName("visualizeIMUs");
    connect(visualizeIMUs, SIGNAL(clicked()), this, SLOT(showIMUs()));
    options1->addWidget(visualizeIMUs);

    QCheckBox *visualizeEstimatedCOM = new QCheckBox(tr("show estimated COM"));
    visualizeEstimatedCOM->setObjectName("visualizeEstimatedCOM");
    connect(visualizeEstimatedCOM, SIGNAL(clicked()), this, SLOT(showEstimatedCOM()));
    options1->addWidget(visualizeEstimatedCOM);

    options->addLayout(options0);
    options->addLayout(options1);
    frameLayout->addLayout(options);

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
    id_sub = nh->subscribe("/roboy/id", 1, &BalancingPlugin::updateId, this);
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

void BalancingPlugin::showForceTorqueSensors(){
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeForceTorqueSensors");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = ForceTorqueSensors;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::showIMUs(){
    QCheckBox* w = this->findChild<QCheckBox*>("visualizeIMUs");
    roboy_simulation::VisualizationControl msg;
    msg.roboyID = currentID.second;
    msg.control = IMUs;
    msg.value = w->isChecked();
    roboy_visualization_control_pub.publish(msg);
}

void BalancingPlugin::changeID(int index){
    QComboBox* roboyID = this->findChild<QComboBox*>("roboyID");
    currentID = make_pair(index, roboyID->currentText().toInt());
    // republish visualization
    showMesh();
    showCOM();
    showEstimatedCOM();
    showForce();
    showTendon();
    showForceTorqueSensors();
    showIMUs();
}

void BalancingPlugin::resetWorld(){
    std_srvs::Trigger srv;
    reset_world_srv.call(srv);
}

void BalancingPlugin::play(){
    std_msgs::Int32 msg;
    msg.data = Play;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::pause(){
    std_msgs::Int32 msg;
    msg.data = Pause;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::slowMotion(){
    std_msgs::Int32 msg;
    msg.data = Slow_Motion;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::updateInteractiveMarker(){
    std_msgs::Int32 msg;
    msg.data = UpdateInteractiveMarker;
    sim_control_pub.publish(msg);
}

void BalancingPlugin::sendMotorControl(){
    roboy_simulation::MotorControl msg;
    msg.roboyID = currentID.second;
    QString m("motor");
    for(uint i=0; i<16; i++){
        QLineEdit* line = this->findChild<QLineEdit *>(m+QString::number(i));
        bool ok = false;
        msg.voltage.push_back(line->text().toFloat(&ok));
        if(!ok)
            line->setText("invalid value");
    }
    motor_control_pub.publish(msg);
}

void BalancingPlugin::refresh(){
    showCOM();
    showEstimatedCOM();
    showForce();
    showForceTorqueSensors();
    showIMUs();
    showMesh();
    showTendon();
}

void BalancingPlugin::updateId(const std_msgs::Int32::ConstPtr &msg){
    QComboBox* roboyID = this->findChild<QComboBox*>("roboyID");
    int index = roboyID->findText(QString::number(msg->data));
    if(index==-1) {
        roboyID->addItem(QString::number(msg->data));
        roboyID->repaint();
        // republish visualization
        showMesh();
        showCOM();
        showEstimatedCOM();
        showForce();
        showTendon();
        showForceTorqueSensors();
        showIMUs();
    }
}

PLUGINLIB_EXPORT_CLASS(BalancingPlugin, rviz::Panel)
