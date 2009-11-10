//
// C++ Implementation: pivcontroller
//
// Description:
//
//
// Author:  <>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PIV.hpp"

namespace controller
{
	PIVController::PIVController()
	{
		PIVController( 0,0,0,0,0,0,0,false );
	}

	PIVController::PIVController ( double _Kp, double _Ki, double _Kv, double _Ts,
	                               double _YMin, double _YMax, double _Kt,
	                               bool _slowPosLoop )
	{
		setGains ( _Kp, _Ki,  _Kv );
		setSamplingTime ( _Ts );
		setOutputLimits ( _YMin,  _YMax );
		setIntegratorWindupCoeff ( _Kt );
		setSlowPosLoop ( _slowPosLoop );
		bSecondSample = false;
		limitDiff = 0;
	}

	PIVController::~PIVController()
	{
	}
}

void Controller::PIVController::setGains ( double _Kp, double _Ki, double _Kv )
{
	Kp = _Kp;
	Ki = _Ki;
	Kv = _Kv;
}

void Controller::PIVController::setSamplingTime ( double _Ts )
{
	Ts = _Ts;
	velITerm.init ( Ts );
}

void Controller::PIVController::setOutputLimits ( double _YMin, double _YMax )
{
	YMin = _YMin;
	YMax = _YMax;
}

void Controller::PIVController::setIntegratorWindupCoeff ( double _Kt )
{
	Kt = _Kt;
}

void Controller::PIVController::setSlowPosLoop ( bool _val )
{
	bSlowPosLoop = _val;
}

double Controller::PIVController::saturate ( double _val )
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

double Controller::PIVController::updateVelLoop ( double _velocity )
{
	return velCommand = saturate(( Kv * _velocity ) + velITerm.update( Ki*(posCommand - _velocity) + Kt*limitDiff ));
}

double Controller::PIVController::updatePosLoop ( double _posError )
{
	if(bSlowPosLoop)
	{
		if(!bSecondSample)	
		{
			bSecondSample = true;
			return posCommand = Kp * (_posError);
		}
		else 
			bSecondSample = false;
	}
	else
		return posCommand = Kp * (_posError);
}

double Controller::PIVController::update ( double _velocity, double _posError )
{
	updatePosLoop(_posError);
	updateVelLoop(_velocity);
	return velCommand;
}




