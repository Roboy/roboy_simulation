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
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();
    jointCommand_pub = nh->advertise<roboy_communication_middleware::JointCommand>("/roboy/middleware/JointCommand", 1);

    // link names
    msg.link_name.push_back("Groin_right");
    msg.link_name.push_back("Groin_left");
    msg.link_name.push_back("Knee_right");
    msg.link_name.push_back("Knee_left");

    // publish the same angle for all joints
    msg.angle.push_back(0);
    msg.angle.push_back(0);
    msg.angle.push_back(0);
    msg.angle.push_back(0);
}

PaBiDanceSimulator::~PaBiDanceSimulator(){}

void PaBiDanceSimulator::publishAngles(float angle)
{
    msg.angle[0] = degreesToRadians(angle);
    msg.angle[1] = degreesToRadians(angle);
    msg.angle[2] = degreesToRadians(angle);
    msg.angle[3] = degreesToRadians(angle);
    jointCommand_pub.publish(msg);
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
