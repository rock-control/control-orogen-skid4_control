#include "PIVController.hpp"
#include <iostream>
#include <math.h>
#include <rtt/NonPeriodicActivity.hpp>


using namespace control;

RTT::NonPeriodicActivity* PIVController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


    PIVController::PIVController(std::string const& name, TaskCore::TaskState initial_state)
: PIVControllerBase(name, initial_state)
{
    refPos = 0;
    refVel = 1.0;

    oPIV.setGains(3.80,0.65,0.07);
    oPIV.setFeedForwardGain(1.00,1.00);
    oPIV.setVelSmoothingGain(0.6);
    oPIV.setSamplingTime(0.001);
    oPIV.setOutputLimits(-0.6,0.6);
    oPIV.setIntegratorWindupCoeff(0.06);
    oPIV.setPositionController(true);

    firstRun = true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PIVController.hpp for more detailed
// documentation about them.

// bool PIVController::configureHook()
// {
//     return true;
// }
bool PIVController::startHook()
{
    for (int i=0;i<4;i++)
    {
	wmcmd.mode[i] 	= hbridge::DM_PWM;
	wmcmd.target[i] = 0;
    }
    return true;
}

void PIVController::updateHook()
{
    // This is the hbridge status
    hbridge::Status status;
    if (! _status.read(status))
    {
	return;
    }   

    wmcmd.stamp	= DFKI::Time::now();
    currTime = wmcmd.stamp.seconds + 1e-6 * wmcmd.stamp.microseconds;

    if(firstRun)
    {
	firstRun = false;

	prevPos  = status.states[0].position;
	prevTime = currTime;

	startTime = currTime;
        
	refVelIntegrator.init(0.001, 0.0, prevPos);

	wmcmd.mode[0] 	= hbridge::DM_PWM;
	return;
    }

    time = currTime - startTime;
    if(time <= 1.0)
	refVel = 2.5 * time; 
    else
	if (time > 1.0 && time <= 2.0)
	    refVel = 2.5 - 2.5 * (time-1.0);
	else
	    refVel = 0.0;

    refVel = 2.0+0.5*sin(8.0*time);
//
//    refVel = 4.0;

    actVel = (status.states[0].position - prevPos) / (currTime - prevTime);

    refPos = refVelIntegrator.update(refVel);
    errPos = refPos - status.states[0].position;
    wmcmd.target[0] = oPIV.update(actVel, refVel, errPos);

    prevPos  = status.states[0].position;
    prevTime = currTime;
//
//    wmcmd.mode[0] = hbridge::DM_SPEED;
//    wmcmd.target[0] = refVel;
    // Writing out the message
    _cmd_out.write(wmcmd);
}

// void PIVController::errorHook()
// {
// }
 void PIVController::stopHook()
 {
    for (int i=0;i<4;i++)
    {
	wmcmd.mode[i] 	= hbridge::DM_PWM;
	wmcmd.target[i] = 0;
    }
 }
// void PIVController::cleanupHook()
// {
// }

