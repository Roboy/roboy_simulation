// by Bradley Snyder
// githandle: bradley219
#include <iostream>
#include <cmath>
#include "roboy_simulation/RobotSimulation/pid.hpp"
#include <ros/ros.h>

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double dt, double setpoint, double pv );

    private:
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID( double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(max,min,Kp,Kd,Ki);
}
double PID::calculate( double dt, double setpoint, double pv )
{
    return pimpl->calculate(dt,setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double max, double min, double Kp, double Kd, double Ki ) :
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double dt,double setpoint, double pv )

{
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    double Iout = _Ki * _integral;

    // Derivative term
    if(dt != 0.0)return 0;
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    ROS_INFO("output: %f", output);
    return output;
}

PIDImpl::~PIDImpl()
{
}