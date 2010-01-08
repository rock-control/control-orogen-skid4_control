#include "PIVController.hpp"
#include <iostream>
#include <math.h>
#include <rtt/NonPeriodicActivity.hpp>

#define SAMPLING_TIME 0.001


// already defines in drivers/hbridge/HBridge.hpp
#define FL 3  // Front Left
#define FR 2  // Front Right
#define RL 0  // Rear Left
#define RR 1  // Rear Right

#define MASTER FL // setting the master wheel

#define _2PI_5 1.2566370614 // Angle between wheel legs
#define _PI_5 0.6283185307  // Half of angle between wheel legs

using namespace control;

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
	oPIV[i].setGains(3.80,0.65,0.07);
	oPIV[i].setFeedForwardGain(1.00,0.00);
	oPIV[i].setVelSmoothingGain(0.6);
	oPIV[i].setSamplingTime(SAMPLING_TIME);
	oPIV[i].setOutputLimits(-0.6,0.6);
	oPIV[i].setIntegratorWindupCoeff(0.06);
	oPIV[i].setPositionController(true);
    }

    for (int i=0;i<4;i++)
    {
	wmcmd.mode[i] 	= hbridge::DM_PWM;
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
    _four_wheel_cmd.clear();
    return true;
}

bool PIVController::validInput(controldev::FourWheelCommand const& refVel) const
{
    for (int i = 0; i < 4; ++i)
    {
	if (refVel.mode[i] != controldev::MODE_SPEED)
	    return false;
	else if (fabs(refVel.target[i]) > 7.0)
	    return false;
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

    if(sync_prev != refVel.sync)
        firstRun = true;

    sync_prev = refVel.sync;

    if(!calibrated && refVel.sync) // Calibrate if not already done
    {
/*	if(!calibrate(status)) // If still not calibrated exit the function
	{
	    _cmd_out.write(wmcmd);
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
	    wmcmd.mode[i] 	= hbridge::DM_PWM;
	    wmcmd.target[i] = 0;
	}
	firstRun = true;
	_cmd_out.write(wmcmd);
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
	    wmcmd.mode[i] 	= hbridge::DM_PWM;
	}
	return;
    }

    if(refVel.sync)
    {
        for(int i=0;i<4;i++)
            refVel.target[i] = refVel.target[MASTER];
    }

    for(int i=0; i<4; i++)
    {
	actVel[i] = (status.states[i].position - prevPos[i]) / ((currIndex - prevIndex) * 0.001);
        refPos[i] = refVelIntegrator[i].update(refVel.target[i]);
	errPos[i] = refPos[i] - status.states[i].position;
	wmcmd.target[i] = oPIV[i].update(actVel[i], refVel.target[i], errPos[i]);
	prevPos[i]  = status.states[i].position;
    }
    prevIndex = currIndex;

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

bool PIVController::calibrate(hbridge::Status status)
{
//    std::cout<<"Entered Calibration "<<std::endl;
    bool reached_maximum = true;

    for (int i = 0; i < 4; i++)
    {
	wmcmd.mode[i] = hbridge::DM_PWM;
	if (still_motor[i] < 300)
	{
	    if(forward)			// doing calibration in the forward direction
		wmcmd.target[i] = 0.1; //Slow PWM in the forward direction         		
	    else
		wmcmd.target[i] = -0.1; //Slow PWM in the reverse direction         		

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
	  //  std::cout << "Forward Calibration Over" << std::endl;
	    for(int i=0;i<4;i++)	
	    {
		init_pos[i] = status.states[i].position; // store initial calibration data
		still_motor[i] = 0; // Re-Initialize to Zero for the reverse direction calibration
	    }
	    forward = false;
	    reached_maximum = false;
	    //std::cout << "Maximum Reset ";
	}
	else
	{	
	    for(int i=0;i<4;i++)
		final_pos[i]= status.states[i].position;// store final calibration data

	    for(int i =0;i<4;++i)
	    {
		mid_pos[i] = (init_pos[i] + final_pos[i])/2;    // Mid position  for each wheel
	//	std::cout<<std::endl<<i<<". initial position = " <<init_pos[i]<< "     Final Position  = "<< final_pos[i] << "   Middle position = "<<mid_pos[i];
	    }	

	    calibrated = true;
	  //  std::cout<<std::endl<<"Calibration Done"<<std::endl;
	}
    }
    return calibrated;
}


void PIVController::setSyncRefPos(hbridge::Status status)
{
    double del[4];  // Stores the delta position
    int mul[4];  // Stores the integer multiples of 2PI/5

    for(int i=0;i<4;i++)
    {
        refPos[i] = status.states[i].position;
        del[i] = refPos[i] - mid_pos[i];	
        mul[i] = (int) del[i] / _2PI_5;
        del[i] -=  mul[i] * _2PI_5;
    }

    if(fabs(del[FR]) >= fabs(del[MASTER]))
	    refPos[FR] = mid_pos[FR] + mul[FR] * _2PI_5 + del[MASTER]  + _PI_5;
    else
	    refPos[FR] = mid_pos[FR] + mul[FR] * _2PI_5 + del[MASTER]  - _PI_5;

    if(fabs(del[RL]) >= fabs(del[MASTER]))
	    refPos[RL] = mid_pos[RL] + mul[RL] * _2PI_5 + del[MASTER]  + _PI_5;
    else
	    refPos[RL] = mid_pos[RL] + mul[RL] * _2PI_5 + del[MASTER]  - _PI_5;

    refPos[RR] = mid_pos[RR] + mul[RR] * _2PI_5 + del[MASTER];
}
