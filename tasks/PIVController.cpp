#include "PIVController.hpp"
#include <iostream>
#include <math.h>
#include <rtt/NonPeriodicActivity.hpp>

#define SAMPLING_TIME 0.001

using namespace control;

RTT::NonPeriodicActivity* PIVController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


	PIVController::PIVController(std::string const& name, TaskCore::TaskState initial_state)
: PIVControllerBase(name, initial_state)
{
	for(int i=0; i<4; i++)
	{
		refPos[i] = 0;

		oPIV[i].setGains(3.80,0.65,0.07);
		oPIV[i].setFeedForwardGain(1.00,0.00);
		oPIV[i].setVelSmoothingGain(0.6);
		oPIV[i].setSamplingTime(SAMPLING_TIME);
		oPIV[i].setOutputLimits(-0.6,0.6);
		oPIV[i].setIntegratorWindupCoeff(0.06);
		oPIV[i].setPositionController(true);
	}
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

	_four_wheel_cmd.read (refVel);

	wmcmd.stamp	= DFKI::Time::now();
	currTime = wmcmd.stamp.seconds + 1e-6 * wmcmd.stamp.microseconds;

	if(firstRun)
	{
		firstRun = false;

		prevTime = currTime;
		startTime = currTime;

		for(int i=0; i<4; i++)
		{
			prevPos[i]  = status.states[i].position;
			refVelIntegrator[i].init(SAMPLING_TIME, 0.0, prevPos[i]);
			wmcmd.mode[i] 	= hbridge::DM_PWM;
		}
		return;
	}

	for(int i=0; i<4; i++)
	{
		actVel[i] = (status.states[i].position - prevPos[i]) / (currTime - prevTime);
		refPos[i] = refVelIntegrator[i].update(refVel.target[i]);
		errPos[i] = refPos[i] - status.states[i].position;

		wmcmd.target[i] = oPIV[i].update(actVel[i], refVel.target[i], errPos[i]);

		prevPos[i]  = status.states[i].position;
	}
	prevTime = currTime;

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

