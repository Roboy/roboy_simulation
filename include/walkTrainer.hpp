// std
#include <cstdlib>
#include <iostream>
#include <thread>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
// ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <interactive_markers/interactive_marker_server.h>
// ros messages
#include <std_msgs/Int32.h>
#include "roboy_simulation/ControllerParameters.h"
#include "roboy_simulation/UpdateControllerParameters.h"
#include "roboy_simulation/Energies.h"
// common definitions
#include "CommonDefinitions.h"
#include "controllerParameters.hpp"
#include "helperClasses.hpp"
// libcmaes
#include "cmaes.h"
#include "simulationControl.hpp"

using namespace gazebo;
using namespace std;
using namespace libcmaes;

#define POPULATION_SIZE 1

class WalkTrainer : public CMAStrategy<CovarianceUpdate>, public SimulationControl{
public:
    WalkTrainer(FitFunc &func, CMAParameters<> &parameters);
    ~WalkTrainer();
    /**
     * initializes numberOfWorlds worlds and populates them with the legs
     * @param numberOfWorlds
     */
    void initializeWorlds(uint numberOfWorlds);
    /** Initializes ControllerParameters */
    void initializeControllerParameters(ControllerParameters &params, physics::ModelPtr parent_model);
    dMat ask();
    void eval(const dMat &candidates, const dMat &phenocandidates=dMat(0,0));
    void tell();
    bool stop();
    ControllerParameters inital_params;
private:
    void updateID(const std_msgs::Int32::ConstPtr &msg);
    transport::NodePtr node;
    transport::PublisherPtr serverControlPub, resetPub;

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_world_srv;
    vector<ros::ServiceClient> control_parameters_srvs, energie_srvs;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    vector<int> roboyIDs;
    vector<physics::WorldPtr> world;
    vector<physics::ModelPtr> model;
    vector<ControllerParameters> controllerParams;
};

int main(int _argc, char **_argv);