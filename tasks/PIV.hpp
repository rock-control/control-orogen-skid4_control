//
// C++ Interface: pivcontroller
//
// Description: 
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef CONTROLLERPIVCONTROLLER_H
#define CONTROLLERPIVCONTROLLER_H

# include "SimpleIntegrator.hpp"

namespace controller
{

	/**
		@author
	*/
	class PIVController
	{
		public:
			PIVController();
			PIVController( double _Kp, double _Ki, double _Kv, double _Ts, 
						   double _YMin = 0, double _YMax = 0, double _Kt = 0, 
						   bool _slowPosLoop = false);

			~PIVController();
			void setGains ( double _Kp, double _Ki, double _Kv ); // Sets the PIIV gains
			void setSamplingTime (double _Ts); 
			void setOutputLimits ( double _YMin, double _YMax ); // Sets the max and min output limits
			void setIntegratorWindupCoeff (double _Kt); // Sets the intergrator wind up coefficients
			void setSlowPosLoop (bool _val); // set the reduction in sampling freqof outer loops
			
			double saturate(double _value); // Saturates the input based on YMin and YMax returns the excess value
			
			double updateVelLoop ( double _velocity ); // update velocity loop
			double updatePosLoop ( double _posError ); // updates position loop

			double update ( double _velocity, double _posError ); // updates both the loops

		private:
			double Kp; // Proportional Position loop
			double Ki; // Intergral Velocity loop
			double Kv; // Proportional Position loop
			
			double Kt; // Anti-integrator-windup coefficient
			double limitDiff; // Stores the difference in the saturation

			double Ts; // Sampling time

			double YMax; // Maximum output value
			double YMin; // Minimum output value

			double posCommand; // output of position loop
			double velCommand; // output of velocity loop

			bool bSlowPosLoop; // Set true for a positioin loop at 1/2 Ts
			bool bSecondSample; // Internally used to slow the outer loop

			SimpleIntegrator velITerm; // Integral term
	};

}

#endif
