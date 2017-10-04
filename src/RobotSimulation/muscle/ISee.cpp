#include "roboy_simulation/RobotSimulation/muscle/ISee.hpp"
#include <ros/ros.h>
using namespace roboy_simulation;

    ISee::ISee()
    {

    };

	double ISee::ElasticElementModel(const double _length0, const double _length, double _stiffness,
										const double _speed, const double _spindleRadius, const double _time) {
		// double realTimeUpdateRate=1000;
		double windingLength = _spindleRadius * _speed * _time;
		double displacement;
		displacement = windingLength + _length - _length0;

		// gzdbg << "displacement: " 
		// 	  << displacement
		// 	  << "\n"
		//          << "windingLength: "
		// 	  << windingLength
		// 	  << "\n";

		double elasticForce;

		if (displacement >= 0) {
			elasticForce = displacement * _stiffness;
		}
		else {
			elasticForce = 0;
		}

		//return _stiffness[0] + (displacement*_stiffness[1]) + (displacement*displacement*_stiffness[2]) +
		//			(displacement*displacement*displacement*_stiffness[3]) ;
		//return displacement*_stiffness[0];
		return elasticForce;

	}

	void ISee::ElasticElementModel(SEE &see, const double &length)
    {
		if( see.expansion > -0.02 ){
        see.expansion = length - see.length0;
		}
		//ROS_INFO("expansion: %f", see.expansion);
        if (see.expansion >= 0)
        {
            see.force=see.expansion*see.stiffness;
        }
        else
        {
            see.force=0;
        }
    }

	void ISee::ElasticElementModel( const double &tendonLength, const double &muscleLength )
    {
		//this calculation of the internal length is based on the real myoMuscle geometry
		double deltaLength = ( muscleLength+internalLength - tendonLength );

		deltaX = c4 - ( (internalLength - deltaLength - length_c1 - length_c2 - c3/std::cos(alpha_2)) / (1/std::cos(alpha_1) + 1/std::cos(alpha_2)) );
		if(deltaX > 0.02) { deltaX = 0.02; }
		if(deltaX < 0){ deltaX = 0; }
		
		length_1 = sqrt( c1*c1 + (c4-deltaX)*(c4-deltaX) );
		length_2 = sqrt( c2*c2 + (c3+c4-deltaX)*(c3+c4-deltaX) );

		alpha_1 = std::atan( c1 / (c4-deltaX) );
		alpha_2 = std::atan( c2 / (c3+c4-deltaX) );

		internalLength = length_c1 + length_1 + length_2 + length_c2;  

		// deltaLength will be zero at this point until the spring reaches its limit. 
		deltaLength = ( muscleLength+internalLength - tendonLength ); 
		if(deltaX >= 0.02) 
				tendonForce = tendonStiffness * deltaLength; 
		else{ tendonForce = 0; }
		//ROS_INFO("deltaX: %f;	_1: %f;		_2:%f", deltaX, toDegree(alpha_1), toDegree(alpha_2) );
		
        if (deltaX >= 0)
        {
            see.force= deltaX*see.stiffness;
        }
        else
        {
            see.force=0;
        }
    }

	void ISee::applyTendonForce( double &_muscleForce , double &_actuatorForce ){
		// since the tendon runs over a spindle the force on both tendons will be equal.
		// Only their horizontal and vertical forces will differ due to the different angles toward the spring
			_muscleForce = _actuatorForce = see.force / ( std::cos(alpha_1) + std::cos(alpha_2) ) + tendonForce;
		
			//check for tendon rip
		if(_muscleForce > 3000){ 
			_muscleForce = _actuatorForce = 0;
		}
		
	}