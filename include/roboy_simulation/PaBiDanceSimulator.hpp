#include <stdlib.h>
#include "ros/ros.h"
#include <roboy_communication_middleware/JointCommand.h>
#include <common_utilities/CommonDefinitions.h>
#include <unistd.h>
#include <chrono>

using namespace std;

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
        boost::shared_ptr<ros::AsyncSpinner> spinner;
        ros::Publisher jointCommand_pub;
        roboy_communication_middleware::JointCommand msg;
        /**
         * Adjusts the joint angles gradually to -90° or to 0° depending on the boolean
         */
        bool adjustPoseGradually(bool goUp);
};

int main(int argc, char** arv);