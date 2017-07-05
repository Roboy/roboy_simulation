#ifndef _GAZEBO_ITENDON_HPP_
#define _GAZEBO_ITENDON_HPP_

#include <vector>
#include <boost/numeric/odeint.hpp>

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

		// c1-c3 are constand values. documentation can be found at ____________________
		double c1, c2, c3;
		// the angles alpha_* describe the angle the tendons attach to the see.element
		double alpha_1, alpha_2;
		// the angles beta_* describe the third angles of the corresponding triangles  
		double beta_1, beta_2;
		// length_* is the tendonlength from the two triangles inside the motor
		double length_1, length_2;
		// length_c* are constand tendonlengths inside the motor
		double length_c1, length_c2;

       public:
		//deltaX is the dispöacement of the spring inside the motor
		double deltaX = 0.0; //m
		// the Length of the tendon inside the motor. the internal length changes depending on the displacement of the spring.
		double internalLength = 0.1; //m // length_c1 + length_1 + length_2 + length_c2;
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
