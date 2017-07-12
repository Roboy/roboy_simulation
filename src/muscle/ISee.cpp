#include "roboy_simulation/muscle/ISee.hpp"
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
		bool complex = true;
		if(complex){//this calculation of the internal length is based on the real myoMuscle geometry
			double deltaLength = ( muscleLength+internalLength - tendonLength );

			deltaX = c4 - ( (internalLength - deltaLength - length_c1 - length_c2 - c3/std::cos(alpha_2)) / (1/std::cos(alpha_1) + 1/std::cos(alpha_2)) );
			if(deltaX > 0.02) { deltaX = 0.02; }
			
			length_1 = sqrt( c1*c1 + (c4-deltaX)*(c4-deltaX) );
			length_2 = sqrt( c2*c2 + (c3+c4-deltaX)*(c3+c4-deltaX) );

			alpha_1 = std::atan( c1 / (c4-deltaX) );
			alpha_2 = std::atan( c2 / (c3+c4-deltaX) );

			internalLength = length_c1 + length_1 + length_2 + length_c2;

			// deltaLength will be zero at this point until the spring reaches its limit.
			deltaLength = ( muscleLength+internalLength - tendonLength );
			tendonForce = tendonStiffness * deltaLength;

		}else{//this is a simple simulation of the internal Length
			auto tmp = (muscleLength+internalLength - tendonLength) / 2.0;
			deltaX = ( 0 < tmp) ? tmp : 0;
			internalLength = 0.1 - 2* deltaX;
		}
		//ROS_INFO("deltaX: %f;	_1: %f;		_2:%f", deltaX, toDegree(alpha_1), toDegree(alpha_2) );
		
        if (deltaX >= 0)
        {
            see.force= deltaX*see.stiffness + tendonForce;
        }
        else
        {
            see.force=0;
        }
    }

	void ISee::applyTendonForce( double &_muscleForce , double &_actuatorForce ){
		// since the tendon runs over a spindle the force on both tendons will be equal.
		// Only their horizontal and vertical forces will differ due to the different angles toward the spring
		_muscleForce = _actuatorForce = see.force / ( std::cos(alpha_1) + std::cos(alpha_2) );
	}
/*
	math::Vector3 ISee::CalculateForce(double _elasticForce, double _motorForce,
										  const math::Vector3 &_tendonOrien) {
		// math::Vector3 diff = _fixationP - _instertionP;

		//    double tendonForce;

		//if (_elasticForce+_motorForce>=0)
		//{
		//	tendonForce=_elasticForce+_motorForce;
		//}
		//else
		//{
		//	tendonForce=0;
		//}

		return _tendonOrien * (_elasticForce + _motorForce);

	}
*/
/*
	void ISee::GetTendonInfo(vector<math::Vector3> &viaPointPos, tendonType *tendon_p)//try later with pointer
	{
		for (int i = 0; i < viaPointPos.size() - 1; i++) {
			tendon_p->MidPoint.push_back((viaPointPos[i] + viaPointPos[i + 1]) / 2);
			tendon_p->Vector.push_back(viaPointPos[i] - viaPointPos[i + 1]);
			tendon_p->Orientation.push_back(tendon_p->Vector[i] / tendon_p->Vector[i].GetLength());
			tendon_p->Pitch.push_back(atan(tendon_p->Orientation[i][0] / tendon_p->Orientation[i][2]));
			tendon_p->Roll.push_back(
					-acos(sqrt((pow(tendon_p->Orientation[i][0], 2) + pow(tendon_p->Orientation[i][2], 2)))));
		}
	}
*/
/*
	double ISee::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2) {
		return _v1.x * _v2.x + _v1.y * _v2.y + _v1.z * _v2.z;
	}


	double ISee::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2) {
		return acos(_v1.Dot(_v2) / _v1.GetLength() * _v2.GetLength());
	}
*/
