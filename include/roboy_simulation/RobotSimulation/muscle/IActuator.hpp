#ifndef _GAZEBO_IACTUATOR_HPP_
#define _GAZEBO_IACTUATOR_HPP_

#include <boost/numeric/odeint.hpp>

namespace gazebo{

    struct Motor {
        double current = 0.0; // [A]
        double torqueConst = 14.2e-03; // [Nm/A]
        double resistance = 0.797 + 0.12; // [Ohm] +0.12 for PWM simulation
        double inductance = 0.118e-03; // [H]
        double voltage = 0.0; // [V]
        double BEMFConst = 14.2e-03; // [V/s]
        double inertiaMoment = 4.09e-06; // [kgm^2]
		double continuousTorque = 47.6e-03; // [Nm]
		double stallTorque	= 857e-03; //[Nm]
		double continuousCurrent = 3.45; // [A]
		double stallCurrent	= 60.2; //[A]
		double speed_torque_gradient = 626.667; // [rps/Nm] = 37.6[rpm/mNm]
	};

	struct Gear {
        double inertiaMoment = 0.4e-07; // [kgm^2]
        double ratio = 53; // [1]
        double efficiency = 0.59; // [1]
        double appEfficiency; // approximated efficiency
        double position = 0.0;
	};

	struct Spindle {
        double angVel = 0.0; // [1/s]
        double radius = 4.5e-03; // [m]
	};

    class IActuator {
		// state vector for differential model
	public:
		typedef std::vector<double> state_type;
		// private: std::vector< double > x(2);

		// stepper for integration
		boost::numeric::odeint::runge_kutta_cash_karp54<state_type> stepper;

		////////////////////////////////////////
		/// \brief Approximates gear's velocity
		/// according to the direction of the rotation of the gear, i.e.
		/// eta or 1/eta
		/// \return Approximated value for gear efficiency
		double EfficiencyApproximation();

		void DiffModel(const state_type &x, state_type &dxdt, const double /* t */);


        ////////////////////////////////////////
		/// \brief Calculate torque for an electric motor model.
		/// \param[in] _current Input electric current
		/// \param[in] _torqueConstant Motor's torque constant
		/// \param[in] _spindleRadius Radius of the spindle that coils up the tendon
		/// \return Calculated force according to the model
		double ElectricMotorModel(const double _current, const double _torqueConstant,
								  const double _spindleRadius, const double _simAngVel);

		Motor motor;
		Gear gear;
		Spindle spindle;
		double elasticForce;
	};
}

#endif
