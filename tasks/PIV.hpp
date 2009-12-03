//
// C++ Interface: PIV 
//
// Description: 
//
//
// Author:  <Ajish Babu>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef CONTROLLERPIVCONTROLLER_H
#define CONTROLLERPIVCONTROLLER_H

# include "SimpleIntegrator.hpp"

namespace controller
{
    class PIVController
    {
	public:
	    PIVController();
	    PIVController( double _Kpp, double _Kiv, double _Kpv, 
		    double _Kvff, double _Kaff,
		    double _Kalp, 
		    double _Ts, 
		    double _YMin = 0, double _YMax = 0, 
		    double _Kt = 0);

	    ~PIVController();

	    void setGains ( double _Kpp, double _Kiv, double _Kpv ); // Sets the PIV gains
	    void setFeedForwardGain( double _Kvff=0.0, double _Kaff=0.0 ){ Kvff = _Kvff; Kaff = _Kaff; };
	    void setVelSmoothingGain( double _Kalp )  { Kalp = _Kalp; };
	    void setSamplingTime (double _Ts); 
	    void setOutputLimits ( double _YMin, double _YMax ){ YMin = _YMin; YMax = _YMax; }; // Sets the max and min output limits
	    void setPositionController(bool _status) { posController = _status; };
	    void setIntegratorWindupCoeff (double _Kt) { Kt = _Kt; }; // Sets the intergrator wind up coefficients

	    double saturate_windup(double _value); // Saturates the input based on YMin and YMax returns the excess value
	    double saturate ( double _val );

	    double updateVelLoop ( double _velMeasured, double _velCmd, double accFF=0.0 ); // update velocity loop
	    double updatePosLoop ( double _posError ){ return posCommand = Kpp * (_posError); }; // updates position loop

	    double update ( double _velMeasured, double _velCmd, double _posError=0.0, double _accFF=0.0  ); // updates both the loops


	private:
	    double Kpp; // Proportional Position loop
	    double Kiv; // Intergral Velocity loop
	    double Kpv; // Proportional Position loop

	    double Kvff; // Velocity feed forward 
	    double Kaff; // Acceleration feed forward
	    double Kalp;  // Smoothing factor for velocity input 0 <= Ka1 < 1    

	    double Kt; // Anti-integrator-windup coefficient
	    double limitDiff; // Stores the difference in the saturation

	    double Ts; // Sampling time

	    double YMax; // Maximum output value
	    double YMin; // Minimum output value

	    double velSmooth; // Filtered velocity
	    double velPrevStep; // Velocity from previous step for filtering

	    double posCommand; // output of position loop
	    double velCommand; // output of velocity loop

	    bool posController; // Activate or deactivate position controller

	    SimpleIntegrator velITerm; // Integral term
    };

}

#endif
