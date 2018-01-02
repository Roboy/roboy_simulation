#include "roboy_simulation/MyoMusclePID.hpp"

MyoMusclePID::MyoMusclePID(){
    getDefaultControlParams(&params[POSITION], POSITION);
    getDefaultControlParams(&params[VELOCITY], VELOCITY);
    getDefaultControlParams(&params[DISPLACEMENT], DISPLACEMENT);
    getDefaultControlParams(&params[FORCE], DISPLACEMENT);
}

double MyoMusclePID::calculate( double dt, double setpoint, double pv ) {
    if(dt == 0.0)
        return 0;

    double result = 0;

    err = setpoint - pv;
    if (fabs(err)>params[control_mode].deadBand) {
        pterm = params[control_mode].Kp * err;
        if ((pterm < params[control_mode].outputPosMax) || (pterm > params[control_mode].outputNegMax)){
            iterm = iterm + (params[control_mode].Ki * err); //add to the iterm
            if (iterm > params[control_mode].IntegralPosMax)
                iterm = params[control_mode].IntegralPosMax;
            else if (iterm < params[control_mode].IntegralNegMax)
                iterm = params[control_mode].IntegralNegMax;
        }
        dterm = ((err - last_error)/dt * params[control_mode].Kd);
        ffterm = (params[control_mode].forwardGain * setpoint);
        result = ffterm + pterm + iterm + dterm;
        ROS_DEBUG_THROTTLE(1,"pterm: %lf iterm: %lf dterm: %lf result: %lf setpoint: %lf pv: %lf", pterm, iterm, dterm, result, setpoint, pv);
    }else{
        result = iterm;
    }

    if ((result < params[control_mode].outputNegMax))
        result = params[control_mode].outputNegMax;
    else if ((result > params[control_mode].outputPosMax))
        result = params[control_mode].outputPosMax;

    last_error = err;

    return result;
}

void MyoMusclePID::getDefaultControlParams(control_Parameters_t *params, int control_mode) {
    params->outputPosMax = 24;
    params->outputNegMax = -24;

    params->radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);

    switch (control_mode) {
        case POSITION:
            params->spPosMax = 10000000;
            params->spNegMax = -10000000;
            params->Kp = 100;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = -100;
            break;
        case VELOCITY:
            params->spPosMax = 100;
            params->spNegMax = -100;
            params->Kp = 100;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = -100;
            break;
        case DISPLACEMENT:
            params->spPosMax = 2000;
            params->spNegMax = 0;
            params->Kp = 100;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = 0;
            break;
        default:
            ROS_ERROR_NAMED("PID controller", "unknown control mode %d", control_mode);
            break;
    }

}