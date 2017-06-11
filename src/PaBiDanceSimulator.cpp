#include "roboy_simulation/PaBiDanceSimulator.hpp"

PaBiDanceSimulator::PaBiDanceSimulator()
{
    // init ros if not initialized
    if(!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "PaBiDanceSimulator", ros::init_options::NoSigintHandler);
    }
    // use NodeHandlePtr instead of NodeHandle so we can use it before ros::init() in the definition of the class
    nh = ros::NodeHandlePtr(new ros::NodeHandle());
    hip1_pub = nh->advertise<std_msgs::Float32>("/roboy/pabi_angle/hip_1", 100);
    hip2_pub = nh->advertise<std_msgs::Float32>("/roboy/pabi_angle/hip_2", 100);
    knee1_pub = nh->advertise<std_msgs::Float32>("/roboy/pabi_angle/knee_1", 100);
    knee2_pub = nh->advertise<std_msgs::Float32>("/roboy/pabi_angle/knee_2", 100);
}

PaBiDanceSimulator::~PaBiDanceSimulator(){}

void PaBiDanceSimulator::publishAngles(float angle)
{
    // publish the same angle for all joints
    std_msgs::Float32 msg;
    msg.data = angle * Degree2Radian;
    hip1_pub.publish(msg);
    hip2_pub.publish(msg);
    knee1_pub.publish(msg);
    knee2_pub.publish(msg);
}

bool PaBiDanceSimulator::adjustPoseGradually(bool goUp)
{
    float stepSize = 1;
    int sleeptime = 10000;
    // adjusts the joint angles to -90° in 90 * stepSize * 0.01 seconds
    if(goUp)
    {
        float currentAngle = 0;
        while(currentAngle > -90)
        {
            publishAngles(currentAngle);
            usleep(sleeptime);
            currentAngle -= stepSize;
        }
    }
    else
    {
        float currentAngle = -90;
        while(currentAngle < 0)
        {
            publishAngles(currentAngle);
            usleep(sleeptime);
            currentAngle += stepSize;
        }
    }
    return true;
}

void PaBiDanceSimulator::startDanceAnimation()
{
    while(ros::ok())
    {
        if(adjustPoseGradually(true))
            adjustPoseGradually(false);
    }
}

int main(int argc, char** argv)
{
    PaBiDanceSimulator paBiDanceSimulator;
    paBiDanceSimulator.startDanceAnimation();
    return 0;
}