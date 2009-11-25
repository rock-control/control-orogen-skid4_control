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
	PIVController( 0,0,0,1,0,0,0,0,0 );
    }

    PIVController::PIVController ( 
	    double _Kpp, double _Kiv, double _Kpv, 
	    double _Kvff, double _Kalp,
	    double _Ts,
	    double _YMin, double _YMax, 
	    double _Kt)
    {
	setGains ( _Kpp, _Kiv,  _Kpv );
	setVelFeedForwardGain( _Kvff );
	setVelSmoothingGain( _Kalp );
	setSamplingTime ( _Ts );
	setOutputLimits ( _YMin,  _YMax );
	setIntegratorWindupCoeff ( _Kt );
	limitDiff = 0.0;
	velPrevStep = 0.0;
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

void controller::PIVController::setVelFeedForwardGain ( double _Kvff )
{
    Kvff = _Kvff;
}

void controller::PIVController::setVelSmoothingGain( double _Kalp )
{
    Kalp = _Kalp;
}

void controller::PIVController::setSamplingTime ( double _Ts )
{
    Ts = _Ts;
    velITerm.init ( Ts );
}

void controller::PIVController::setOutputLimits ( double _YMin, double _YMax )
{
    YMin = _YMin;
    YMax = _YMax;
}

void controller::PIVController::setIntegratorWindupCoeff ( double _Kt )
{
    Kt = _Kt;
}

double controller::PIVController::saturate ( double _val )
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

double controller::PIVController::updateVelLoop ( double _velMeasured, double _velCmd )
{
    velSmooth = (1-Kalp)*_velMeasured + Kalp*velPrevStep;
//    std::cout << "\n velSmooth: " << velSmooth;
    velPrevStep = velSmooth;
//    std::cout << ",  velPrevStep: " << velPrevStep;
    velCommand = (Kvff * _velCmd) + posCommand - velSmooth;
//    std::cout << ",  velControllerInput: " << velCommand;
    velCommand = saturate((Kpv*velCommand) + velITerm.update(Kiv*velCommand + Kt*limitDiff));
//    std::cout << ",  Command: " << velCommand;
    return velCommand;
}

double controller::PIVController::updatePosLoop ( double _posError )
{
    return posCommand = Kpp * (_posError);
}

double controller::PIVController::update ( double _velMeasured, double _velCmd, double _posError )
{
    updatePosLoop(_posError);
    return updateVelLoop(_velMeasured, _velCmd);
}
