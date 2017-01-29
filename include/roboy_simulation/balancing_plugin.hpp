#pragma once

#ifndef Q_MOC_RUN
// qt
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
#include <QGroupBox>
// std
#include <map>
// ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//messages
#include "roboy_simulation/VisualizationControl.h"
#include "roboy_simulation/LegState.h"
#include "roboy_simulation/ControllerParameters.h"
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "roboy_simulation/Abortion.h"
#include "roboy_simulation/MotorControl.h"
// common definitions
#include "common_utilities/CommonDefinitions.h"
#endif

using namespace std;

class BalancingPlugin : public rviz::Panel{
Q_OBJECT

public:
    BalancingPlugin(QWidget *parent = 0);

    ~BalancingPlugin();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

public Q_SLOTS:
    /** Sends a message that triggers publishing of COM visualization */
    void showCOM();
    /** Sends a message that triggers publishing of estimated COM visualization */
    void showEstimatedCOM();
    /** Sends a message that triggers publishing of forces from the muscles visualization*/
    void showForce();
    /** Sends a message that triggers publishing of Tenodon visualization */
    void showTendon();
    /** Sends a message that triggers publishing of mesh visualization */
    void showMesh();
    /** Sends a message that triggers publishing of ankle force visualization */
    void showForceTorqueSensors();
    /** Sends a message that triggers publishing of IMU visualization */
    void showIMUs();
    /** Sends a message that triggers filtering of IMU data */
    void toggleIMUFiltering();
    /** Call to reset world service */
    void resetWorld();
    /** Sends a play message */
    void play();
    /** Sends a pause message */
    void pause();
    /** Sends a slow motion */
    void slowMotion();
    /** Refreshes visualization updates */
    void refresh();

private:

    ros::NodeHandle *nh;
    pair<uint, uint> currentID;
    map<uint, ros::Subscriber> leg_state_sub;
    ros::AsyncSpinner *spinner;
    ros::Publisher roboy_visualization_control_pub, sim_control_pub, motor_control_pub;
    ros::Subscriber simulation_state_sub;
    ros::ServiceClient reset_world_srv;

    ros::Timer frame_timer;
};
