#include "roboy_simulation/RobotSimulation/muscle/IActuator.hpp"

using namespace gazebo;

double IActuator::EfficiencyApproximation() {
    double param1 = 0.1; // defines steepness of the approximation
	double param2 = 0; // defines zero crossing of the approximation
	return gear.efficiency + (1 / gear.efficiency - gear.efficiency) *
								 (0.5 * (tanh(-param1 * spindle.angVel * motor.current - param2) + 1));
}

	double IActuator::ElectricMotorModel(const double _current, const double _torqueConstant,
									   const double _spindleRadius, const double _simAngVel) {
		double motorForce;

		if (_current >= 0) {
			//force at the motor
			motorForce = _current * _torqueConstant;// - motor.inertiaMoment *(_simAngVel * gear.ratio)* (_simAngVel * gear.ratio);
			//force after the gear
			motorForce *= gear.ratio;// - gear.inertiaMoment *_simAngVel* _simAngVel;
			//Force left after turning the motor
			motorForce -= motor.speed_torque_gradient * _simAngVel;
			// lineat Force after the spindle
			motorForce /= _spindleRadius;
		}
		else {
			motorForce = 0;
		}

		return motorForce;
	}
