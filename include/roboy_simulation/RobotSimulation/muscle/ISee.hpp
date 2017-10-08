#ifndef _GAZEBO_ITENDON_HPP_
#define _GAZEBO_ITENDON_HPP_

#include <vector>
#include <boost/numeric/odeint.hpp>
#include <cmath>
# define M_PI           3.14159265358979323846  /* pi */
#define toRadian( x )	( (x) / 180 * M_PI )
#define toDegree( x )	( (x) / M_PI * 180 )
namespace roboy_simulation
{
    using namespace std;
	using namespace boost::numeric::odeint;
	//using namespace gazebo;
/*
    struct tendonType {
		vector<math::Vector3> MidPoint;
		vector<math::Vector3> Vector;
		//might need it to calculate length
		vector<math::Vector3> Orientation;
		vector<double> Pitch;
		vector<double> Roll;
	};
*/
	struct SEE {
		double stiffness = 30680.0; // N/m
		double length = 0.056; //m
		double expansion = 0.0; //m
		double force = 0.0;  //
		double length0 = 0; //m
	};
		
		
    class ISee {

		// c1-c3 are constand values, c4 is x0. documentation can be found at ____________________
		double c1 = 0.012, c2 = 0.008, c3 = 0.018, c4 = 0.039; //m
		// the angles alpha_* describe the angle the tendons attach to the see.element
		double alpha_1 = std::atan( c1 / c4 ), alpha_2 = std::atan( c2 /  (c3+c4) ); // radian
		// the angles beta_* describe the third angles of the corresponding triangles  
		double beta_1  =  M_PI / 4 - alpha_1, beta_2 = M_PI / 4 - alpha_2; // radian
		// length_* is the tendonlength from the two triangles inside the motor
		double length_1 = sqrt( c1*c1 + c4*c4 ), length_2 = sqrt( c2*c2 + (c3+c4)*(c3+c4) ); //m
		// length_c* are constand tendonlengths inside the motor
		double length_c1 = 0.04, length_c2 = 0.013; //m
		// Tendon stiffness (this is a random high number. A real number still has to be set) 
		double tendonStiffness = 1e6; // N/m 
		double tendonForce = 0; 

       public:
		//deltaX is the displacement of the spring inside the motor
		double deltaX = 0.0; //m
		// the Length of the tendon inside the motor. the internal length changes depending on the displacement of the spring.
		double internalLength = length_c1 + length_1 + length_2 + length_c2; //m
        SEE see;


        ISee();

		////////////////////////////////////////
		/// \brief Calculate elastic force of the series elastic element
		/// \param[in] _length0 Resting length of the SEE
		/// \param[in] _length Current length of the SEE
		/// \param[in] _stiffness Deafault values for stiffness of the SEE
		/// \return Elastic force in N
		double ElasticElementModel(const double _length0, const double _length,
								   double _stiffness, const double _speed,
								   const double _spindleRadius, const double _time);

        ////////////////////////////////////////
	    /// \brief Calculate elastic force of the series elastic element
	    /// \param[in] see the series elastic element
	    /// \param[in] _length Current length of the spring
	    void ElasticElementModel(SEE &see, const double &length);

		///////////////////////////////////////
		/// \brief Calculate elastic force of the series elastic element
		/// \param[in] The tandonLength represents the length of the entire tendon.from the motor to the last viapoint.
		/// \param[in] The muscleLength representes the length of the tendon forn the outside of the motor to the last viapoint.
		void ElasticElementModel(const double &tendonLength, const double &muscleLength);

		///////////////////////////////////////
		/// \brief apply the springForce onto the tendons going to motor and out the muscle. The force depends on the angle the tendons have towards the spring
		/// \parm[in] the force going out the muscle
		/// \parm[in] the force going toward the motor
		void applyTendonForce( double &_muscleForce , double &_actuatorForce );

		//static void GetTendonInfo(vector<math::Vector3> &viaPointPos, tendonType *tendon_p);
/*
	private:
		////////////////////////////////////////
		/// \brief Calculate the dot product between two vectors
		/// \param[in] _v1 vector 1 coordinates
		/// \param[in] _v2 vector 2 coordinates
		/// \return Dot product
		double DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2);

		////////////////////////////////////////
		/// \brief Calculate the angle between two vectors
		/// \param[in] _v1 vector 1 coordinates
		/// \param[in] _v2 vector 2 coordinates
		/// \return Angle between two vectors in radians
		double Angle(const math::Vector3 &_v1, const math::Vector3 &_v2);
*/
	};
}

#endif
