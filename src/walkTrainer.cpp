#include <thread>
#include "walkTrainer.hpp"

WalkTrainer::WalkTrainer(FitFunc &func, CMAParameters<> &parameters):CMAStrategy<CovarianceUpdate>(func,parameters) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "WalkTrainer", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    initializeWorlds(15);

    controllerParams.resize(15);
};

WalkTrainer::~WalkTrainer() {
}

void WalkTrainer::initializeWorlds(uint numberOfWorlds) {
    // load numberOfWorlds empty worlds
    for (uint i = 0; i < numberOfWorlds; i++) {
        world.push_back(loadWorld("worlds/contact.world"));
    }
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    // load the legs in each world
    for (uint i = 0; i < numberOfWorlds; i++) {
        physics::ModelPtr m = loadModel(world[i], "model://legs_with_muscles_simplified" );
        if (m != nullptr) {
            model.push_back(m);
            roboyIDs.push_back(i);
            char topic[200];
            sprintf(topic, "/roboy%d/controller_parameters", i);
            control_parameters_srvs.push_back( nh->serviceClient<roboy_simulation::UpdateControllerParameters>(topic));
            sprintf(topic, "/roboy%d/energies", i);
            energie_srvs.push_back( nh->serviceClient<roboy_simulation::Energies>(topic));
        }
    }
}

void WalkTrainer::initializeControllerParameters(ControllerParameters &params, physics::ModelPtr parent_model) {
    math::Vector3 euler = parent_model->GetLink("hip")->GetWorldPose().rot.GetAsEuler();
    params[phi_trunk_0] = euler.x;
    params[theta_trunk_0] = euler.y;

    euler = parent_model->GetLink("thigh_left")->GetWorldPose().rot.GetAsEuler();
    params[phi_groin_0 + LEG::LEFT] = euler.x;
    params[theta_groin_0 + LEG::LEFT] = euler.y;

    euler = parent_model->GetLink("thigh_right")->GetWorldPose().rot.GetAsEuler();
    params[phi_groin_0 + LEG::RIGHT] = euler.x;
    params[theta_groin_0 + LEG::RIGHT] = euler.y;

    params[k_v] = 0.1;
    params[k_h] = 0.1;
    params[k_p_theta_left + Stance] = 0.1;
    params[k_p_theta_left + Lift_off] = 0.1;
    params[k_p_theta_left + Swing] = 0.1;
    params[k_p_theta_left + Stance_Preparation] = 0.1;
    params[k_p_theta_right + Stance] = 0.1;
    params[k_p_theta_right + Lift_off] = 0.1;
    params[k_p_theta_right + Swing] = 0.1;
    params[k_p_theta_right + Stance_Preparation] = 0.1;
    params[k_d_theta_left + Stance] = 0.1;
    params[k_d_theta_left + Lift_off] = 0.1;
    params[k_d_theta_left + Swing] = 0.1;
    params[k_d_theta_left + Stance_Preparation] = 0.1;
    params[k_d_theta_right + Stance] = 0.1;
    params[k_d_theta_right + Lift_off] = 0.1;
    params[k_d_theta_right + Swing] = 0.1;
    params[k_d_theta_right + Stance_Preparation] = 0.1;
    params[k_p_phi + LEG::LEFT] = 0.1;
    params[k_p_phi + LEG::RIGHT] = 0.1;
    params[k_d_phi + LEG::LEFT] = 0.1;
    params[k_d_phi + LEG::RIGHT] = 0.1;
    // target force torque gains
    params[k_V] = 0.1;
    params[k_P] = 0.1;
    params[k_Q] = 0.1;
    params[k_omega] = 0.1;
}

dMat WalkTrainer::ask() {
    return CMAStrategy<CovarianceUpdate>::ask();
}

void WalkTrainer::eval(const dMat &candidates,
                       const dMat &phenocandidates) {
    // set the controller parameters of each roboy instantiation
    for (int roboyID = 0; roboyID < candidates.cols(); roboyID++) {
        // get the controller parameters
        _solutions.get_candidate(roboyID).set_x(candidates.col(roboyID));
        roboy_simulation::UpdateControllerParameters msg;
        controllerParams[roboyID] = _solutions.get_candidate(roboyID).get_x();
        controllerParametersToMessage(controllerParams[roboyID], msg.request.params);
        // send them to the roboy instance
        control_parameters_srvs[roboyID].call(msg);
        // update the world
        gazebo::runWorld(world[roboyID], 1);
        // retrieve energies
        roboy_simulation::Energies msg2;
        energie_srvs[roboyID].call(msg2);
        double total_energie = msg2.response.E_speed + msg2.response.E_headori + msg2.response.E_effort;
        _solutions.get_candidate(roboyID).set_fvalue(total_energie);
        //std::cerr << "candidate x: " << _solutions.get_candidate(r).get_x_dvec().transpose() << std::endl;
    }
    update_fevals(candidates.cols());
}

void WalkTrainer::tell() {
    CMAStrategy<CovarianceUpdate>::tell();
}

bool WalkTrainer::stop() {
    return CMAStrategy<CovarianceUpdate>::stop();
}

int main(int _argc, char **_argv) {
    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    int dim = TOTAL_NUMBER_CONTROLLER_PARAMETERS; // problem dimensions.
    std::vector<double> x0(dim, 0.1);
    double sigma = 0.1;
    CMAParameters<> cmaparams(x0, sigma, POPULATION_SIZE);
    FitFunc dummyFunc = [](const double *x, const int N){ return 0; };
    ESOptimizer<WalkTrainer, CMAParameters<>> optim(dummyFunc, cmaparams);
    while (!optim.stop()) {
        dMat candidates = optim.ask();
        optim.eval(candidates);
        optim.tell();
        optim.inc_iter(); // important step: signals next iteration.
    }
    std::cout << optim.get_solutions() << std::endl;

//    interactive_marker_server.reset(new interactive_markers::InteractiveMarkerServer("WalkTrainer"));

    gazebo::shutdown();
}