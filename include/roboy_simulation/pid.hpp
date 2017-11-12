// by Bradley Snyder
// githandle: bradley219

#pragma once

#include <ros/ros.h>
#include <iostream>
#include <cmath>

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double max, double min, double Kp, double Kd, double Ki );

        // dt -  loop interval time
        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double dt, double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};
