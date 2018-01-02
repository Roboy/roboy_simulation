#pragma once

#include <ros/ros.h>
#include <common_utilities/CommonDefinitions.h>

class MyoMusclePID {
    public:
        MyoMusclePID( );

        /**
         * Calculates PID output
         * @param dt diff time
         * @param setpoint
         * @param pv current measured value
         * @return PID output
         */
        double calculate( double dt, double setpoint, double pv );

        control_Parameters_t params[NUMBER_OF_CONTROL_MODES];
        int control_mode = DISPLACEMENT;
    private:
        void getDefaultControlParams(control_Parameters_t *params, int control_mode);

        double pterm = 0, iterm = 0, dterm = 0, ffterm = 0, err = 0, last_error = 0;
};
