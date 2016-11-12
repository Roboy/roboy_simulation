#include "simulationControl.hpp"

int main(int _argc, char **_argv) {
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    SimulationControl sim_control;
    physics::WorldPtr world = sim_control.loadWorld("worlds/empty.world");
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    physics::ModelPtr model = sim_control.loadModel(world, "model://legs_with_upper_body");
    while(ros::ok()){
        sim_control.simulate(world);
    }

    gazebo::shutdown();
}