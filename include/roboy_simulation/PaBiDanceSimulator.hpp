#include <stdlib.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <unistd.h>
#include <chrono>

using namespace std;
/**
 * Constant to convert angles from degrees to radian via multiplication
 */
const double Degree2Radian = 3.14159265359 / 180;

class PaBiDanceSimulator
{
    public:
        PaBiDanceSimulator();
        ~PaBiDanceSimulator();
        /**
         * Publishes angles for all pabi joints
         */
        void publishAngles(float angle);
        /**
         * Starts the dance animation loop.
         */
        void startDanceAnimation();
    private:
        ros::NodeHandlePtr nh;
        ros::Publisher hip1_pub;
        ros::Publisher hip2_pub;
        ros::Publisher knee1_pub;
        ros::Publisher knee2_pub;
        /**
         * Adjusts the joint angles gradually to -90° or to 0° depending on the boolean
         */
        bool adjustPoseGradually(bool goUp);
};

int main(int argc, char** arv);