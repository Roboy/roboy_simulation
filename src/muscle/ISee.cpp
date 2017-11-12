#include "roboy_simulation/muscle/ISee.hpp"

using namespace roboy_simulation;

    ISee::ISee(){
	};

	void ISee::ElasticElementModel( const double &tendonLength, const double &muscleLength )
    {
		//this calculation of the internal length is based on the real myoMuscle geometry
		double deltaLength = ( tendonLength - (muscleLength+internalLength) );

		//update internes length
		internalLength += deltaLength;
		
		//update spring displacement
		deltaX = c4 - ( (internalLength - length_c1 - length_c2 - c3/std::cos(alpha_2)) / (1/std::cos(alpha_1) + 1/std::cos(alpha_2)) );

		//check for physical limits.
		if(deltaX > 0.02) { deltaX = 0.02; }
		if(deltaX < 0){ deltaX = 0; }
		
		//update geometry angles
		alpha_1 = std::atan( c1 / (c4-deltaX) );
		alpha_2 = std::atan( c2 / (c3+c4-deltaX) );  
		
		length_1 = sqrt( c1*c1 + (c4-deltaX)*(c4-deltaX) );
		length_2 = sqrt( c2*c2 + (c3+c4-deltaX)*(c3+c4-deltaX) );

		//actual internal length after limit check
		internalLength = length_c1 + length_c2 + length_1 + length_2;

		// check wether SEE absorbed deltaLength completely 
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
		//if(_muscleForce > 3000){ 
		//	_muscleForce = _actuatorForce = 0;
		//}
		
	}