#include "PIVController.hpp"
#include <iostream>
#include <math.h>

#include <rtt/NonPeriodicActivity.hpp>

#define SAMPLING_TIME   0.001
#define CALIB_SPEED_PWM 0.12 // PWM speed during calibration
#define CALIB_WAIT_TIME 300  // Calibration time in ticks

using namespace control;
using namespace hbridge;
using namespace controldev;

RTT::NonPeriodicActivity* PIVController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


    PIVController::PIVController(std::string const& name, TaskCore::TaskState initial_state)
: PIVControllerBase(name, initial_state)
{
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
    for(int i=0; i<4; i++)
    {
	oPIV[i].setGains(MOTOR.POS_P,MOTOR.VEL_I,MOTOR.VEL_P);
	oPIV[i].setFeedForwardGain(MOTOR.VEL_FF,MOTOR.ACC_FF);
	oPIV[i].setVelSmoothingGain(MOTOR.VEL_SMOOTH);
	oPIV[i].setSamplingTime(SAMPLING_TIME);
	oPIV[i].setOutputLimits(-0.6,0.6);
	oPIV[i].setIntegratorWindupCoeff(MOTOR.INT_WIND_UP);
	oPIV[i].setPositionController(true);
    }

    oRamp.setInitialData(0.0);
    oRamp.setFinalData(MOTOR.POS_P);
    oRamp.setDeltaTime(10000.0); // 5 sec
    oRamp.setType(0);  // Linear

    for (int i=0;i<4;i++)
    {
	wmcmd.mode[i] 	= DM_PWM;
	wmcmd.target[i] = 0;
    }
    firstRun = true;
    sync_prev = false;
    forward = true;
    calibrated = false;
    for (int i = 0; i < 4; ++i)
    {
	still_motor[i] = 0;			//Initializing all the variables
	last_pos[i] = 0;
    }

    _status.clear();
    _four_wheel_command.clear();
    return true;
}

bool PIVController::validInput(FourWheelCommand const& refVel) const
{
    for (int i = 0; i < 4; ++i)
    {
	if (refVel.mode[i] != MODE_SPEED)
	    return false;
	else if (fabs(refVel.target[i]) > 7.0)
	    return false;
    }
    return true;
}

// Converts motion command to four wheel command
void PIVController::motionToFourWheelCmd()
{
    // The output of this controller is a speed command.
    refVel.mode[0] = refVel.mode[1] =
        refVel.mode[2] = refVel.mode[3] = MODE_SPEED;

    double fwd_velocity = mcmd.translation / ROBOT.WHEEL_RADIUS_EFF;
    double differential = mcmd.rotation * ROBOT.TRACK / ROBOT.WHEEL_RADIUS_EFF;
    refVel.target[ROBOT.FRONT_LEFT ] = fwd_velocity - differential;
    refVel.target[ROBOT.REAR_LEFT  ] = fwd_velocity - differential;
    refVel.target[ROBOT.FRONT_RIGHT] = fwd_velocity + differential;
    refVel.target[ROBOT.REAR_RIGHT ] = fwd_velocity + differential;

    for(int i=0; i<4; i++)
        if (refVel.target[i] > 7.0)
                refVel.target[i] = 7.0;
        else  if (refVel.target[i] < -7.0)
                refVel.target[i] = -7.0;
    
    // Check if the robot is going straight
    if(mcmd.rotation >= -0.01 && mcmd.rotation <= 0.01 && (mcmd.translation <= -0.05 || mcmd.translation >= 0.05) )
	refVel.sync = true;
    else
	refVel.sync = false;
}

void PIVController::updateHook()
{
    // This is the hbridge status
    Status status;
    if (! _status.read(status))
    {
	return;
    }  

    if(_four_wheel_command.connected())
	_four_wheel_command.read (refVel);
    else
    {
        _motion_command.read(mcmd);
	motionToFourWheelCmd();
    }

    if(sync_prev != refVel.sync)
    {
    //    if(refVel.sync == true)
	oRamp.reset();
        firstRun = true;
    }

    sync_prev = refVel.sync;

    if(!calibrated && refVel.sync) // Calibrate if not already done
    {
	/*if(!calibrate(status)) // If still not calibrated exit the function
	{
	    _simple_command.write(wmcmd);
	    return;
	}
        else
        {
            firstRun = true; 
        }*/
        for(int i=0;i<4;i++)
            mid_pos[i] = status.states[i].position;
        calibrated = true;
        firstRun = true;
    }

    // Validate the input. If it is not valid, stop the wheels
    if (! validInput(refVel))
    {
	for (int i=0;i<4;i++)
	{
	    wmcmd.mode[i] 	= DM_PWM;
	    wmcmd.target[i] = 0;
	}
	firstRun = true;
	_simple_command.write(wmcmd);
	return;
    }

    wmcmd.time	= base::Time::now();
    currIndex = status.index;
    if(firstRun)
    {
        if(refVel.sync)
            setSyncRefPos(status);

	firstRun = false;
	prevIndex  = currIndex;
	for(int i=0; i<4; i++)
	{
            prevPos[i]  = status.states[i].position;
            if(refVel.sync)
                refVelIntegrator[i].init(SAMPLING_TIME,0.0,refPos[i]);
            else
                refVelIntegrator[i].init(SAMPLING_TIME,0.0,prevPos[i]);
	    oPIV[i].reset();
	    wmcmd.mode[i] 	= DM_PWM;
	}
	return;
    }

    if(refVel.sync)
    {
        for(int i=0;i<4;i++)
            refVel.target[i] = refVel.target[ROBOT.FRONT_LEFT];
    }

    for(int i=0; i<4; i++)
    {	
	oPIV[i].setGains(oRamp.getVal(currIndex),MOTOR.VEL_I,MOTOR.VEL_P);
	actVel[i] = (status.states[i].position - prevPos[i]) / ((currIndex - prevIndex) * 0.001);
        refPos[i] = refVelIntegrator[i].update(refVel.target[i]);
	errPos[i] = refPos[i] - status.states[i].position;
	wmcmd.target[i] = oPIV[i].update(actVel[i], refVel.target[i], errPos[i]);
	prevPos[i]  = status.states[i].position;
    }
    prevIndex = currIndex;

    // Writing out the message
    _simple_command.write(wmcmd);
}

// void PIVController::errorHook()
// {
// }
void PIVController::stopHook()
{
    for (int i=0;i<4;i++)
    {
	wmcmd.mode[i] 	= DM_PWM;
	wmcmd.target[i] = 0;
    }
}
// void PIVController::cleanupHook()
// {
// }

bool PIVController::calibrate(Status status)
{
    bool reached_maximum = true;
    for (int i = 0; i < 4; i++)
    {
	wmcmd.mode[i] = DM_PWM;
	if (still_motor[i] < CALIB_WAIT_TIME)
	{
	    if(forward)			// doing calibration in the forward direction
		wmcmd.target[i] = CALIB_SPEED_PWM; //Slow PWM in the forward direction         		
	    else
		wmcmd.target[i] = -CALIB_SPEED_PWM; //Slow PWM in the reverse direction         		

	    if (fabs(last_pos[i] - status.states[i].position) < 0.001)
		still_motor[i]++;
	    else
		still_motor[i] = 0;

	    reached_maximum = false;
	    last_pos[i] = status.states[i].position;
	}
	else
	    wmcmd.target[i] = 0;			
    }

    if (reached_maximum)
    {
	if (forward)
	{	
	    for(int i=0;i<4;i++)	
	    {
		init_pos[i] = status.states[i].position; // store initial calibration data
		still_motor[i] = 0; // Re-Initialize to Zero for the reverse direction calibration
	    }
	    forward = false;
	    reached_maximum = false;
	}
	else
	{	
	    for(int i=0;i<4;i++)
		final_pos[i]= status.states[i].position;// store final calibration data

	    for(int i =0;i<4;++i)
		mid_pos[i] = (init_pos[i] + final_pos[i])/2;    // Mid position  for each wheel

	    calibrated = true;
	}
    }
    return calibrated;
}


void PIVController::setSyncRefPos(Status status)
{
    double del[4];  // Stores the delta position
    int mul[4];  // Stores the integer multiples of 2PI/5

    for(int i=0;i<4;i++)
    {
        refPos[i] = status.states[i].position;
        del[i] = refPos[i] - mid_pos[i];	
        mul[i] = (int) del[i] / MOTOR._2PI_5;
        del[i] -=  mul[i] * MOTOR._2PI_5;
    }

    if(fabs(del[ROBOT.FRONT_RIGHT]) >= fabs(del[ROBOT.FRONT_LEFT]))
	    refPos[ROBOT.FRONT_RIGHT] = mid_pos[ROBOT.FRONT_RIGHT] + mul[ROBOT.FRONT_RIGHT] * MOTOR._2PI_5 + del[ROBOT.FRONT_LEFT]  + MOTOR._PI_5;
    else
	    refPos[ROBOT.FRONT_RIGHT] = mid_pos[ROBOT.FRONT_RIGHT] + mul[ROBOT.FRONT_RIGHT] * MOTOR._2PI_5 + del[ROBOT.FRONT_LEFT]  - MOTOR._PI_5;

    if(fabs(del[ROBOT.REAR_LEFT]) >= fabs(del[ROBOT.FRONT_LEFT]))
	    refPos[ROBOT.REAR_LEFT] = mid_pos[ROBOT.REAR_LEFT] + mul[ROBOT.REAR_LEFT] * MOTOR._2PI_5 + del[ROBOT.FRONT_LEFT]  + MOTOR._PI_5;
    else
	    refPos[ROBOT.REAR_LEFT] = mid_pos[ROBOT.REAR_LEFT] + mul[ROBOT.REAR_LEFT] * MOTOR._2PI_5 + del[ROBOT.FRONT_LEFT]  - MOTOR._PI_5;

    refPos[ROBOT.REAR_RIGHT] = mid_pos[ROBOT.REAR_RIGHT] + mul[ROBOT.REAR_RIGHT] * MOTOR._2PI_5 + del[ROBOT.FRONT_LEFT];
}

