//
// C++ Implementation: PIV
//
// Description:
//
//
// Author:  <Ajish Babu>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PIV.hpp"
#include <iostream>

namespace controller
{
    PIVController::PIVController()
    {
	PIVController( 0,0,0,0,0,0,0,0,0,0 );
    }

    PIVController::PIVController ( 
	    double _Kpp, double _Kiv, double _Kpv, 
	    double _Kvff, double _Kaff,
	    double _Kalp,
	    double _Ts,
	    double _YMin, double _YMax, 
	    double _Kt)
    {
	setGains ( _Kpp, _Kiv,  _Kpv );
	setFeedForwardGain( _Kvff, _Kaff );
	setVelSmoothingGain( _Kalp );
	setSamplingTime ( _Ts );
	setOutputLimits ( _YMin,  _YMax );
	setIntegratorWindupCoeff ( _Kt );
	limitDiff = 0.0;
	velPrevStep = 0.0;
	setPositionController(true);
    }

    PIVController::~PIVController()
    {
    }
}

void controller::PIVController::setGains ( double _Kpp, double _Kiv, double _Kpv )
{
    Kpp = _Kpp;
    Kiv = _Kiv;
    Kpv = _Kpv;
}

void controller::PIVController::setSamplingTime ( double _Ts )
{
    Ts = _Ts;
    velITerm.init ( Ts );
}

double controller::PIVController::saturate_windup ( double _val )
{
    if ( _val > YMax )
    {
	limitDiff = YMax - _val;
	return YMax;
    }
    else if ( _val < YMin )
    {
	limitDiff = YMin - _val;
	return YMin;
    }
    else 
	return _val;
}
double controller::PIVController::saturate ( double _val )
{
    if ( _val > YMax )
	return YMax;
    else if ( _val < YMin )
	return YMin;
    else 
	return _val;
}

double controller::PIVController::updateVelLoop ( double _velMeasured, double _velCmd, double _accFF )
{
    velSmooth = (1-Kalp)*_velMeasured + Kalp*velPrevStep;
    velPrevStep = velSmooth;
    velCommand = (Kvff * _velCmd) + posCommand - velSmooth;
    velCommand = saturate_windup((Kpv*velCommand) + velITerm.update(Kiv*velCommand + Kt*limitDiff));
    velCommand = saturate(velCommand + Kaff * _accFF); 
    return velCommand;
}

double controller::PIVController::update ( double _velMeasured, double _velCmd, double _posError, double _accFF )
{
    if(posController)
   	 updatePosLoop(_posError);
    else 
	 posCommand = 0.0;
    return updateVelLoop(_velMeasured, _velCmd, _accFF);
}
