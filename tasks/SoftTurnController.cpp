/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SoftTurnController.hpp"
#include "motor_controller/PID.hpp"
#include <base/actuators/vehicles.h>
#include <base/commands/Motion2D.hpp>

using namespace skid4_control;

SoftTurnController::SoftTurnController(std::string const& name)
    : SoftTurnControllerBase(name)
{
}

SoftTurnController::~SoftTurnController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SoftTurnController.hpp for more detailed
// documentation about them.

bool SoftTurnController::configureHook()
{
    if (! SoftTurnControllerBase::configureHook())
        return false;
    
    double m_radius = _wheel_radius.get();
    double m_track_width = _track_width.get();
    if (m_radius <= 0 || m_track_width <= 0)
    {
        RTT::log(RTT::Error) << "wrong radius and/or track_width parameters" << RTT::endlog();
        return false;
    }
    
    turnSpeed = _turnSpeed.get();
    translationalSpeed = _translationalSpeed.get();
    state = FORWARD;
    ///maximum allowed displacement while turning
    turnVariance = _turnVariance.get();

    startStatus.resize(4);

    Kp = _P.get();
    Ki = _I.get();
    Kd = _D.get();
    N = _N.get();
    B = _B.get();
    Tt = _Tt.get();
    YMin = -_MinMax.get();
    YMax = _MinMax.get();
    leftPid.setParallelCoefficients(Kp, Ki, Kd, N, B, Tt, YMin, YMax);
    rightPid.setParallelCoefficients(Kp, Ki, Kd, N, B, Tt, YMin, YMax);
    
    return true;
}
// bool SoftTurnController::startHook()
// {
//     if (! SoftTurnControllerBase::startHook())
//         return false;
//     return true;
// }


void currentController(double input)
{
    
}


double SoftTurnController::leftController(double wantedSpeed, const base::actuators::Status &status)
{    
    static double lastOutput = 0.0;
    const double timeSinceLastMeasurement = status.index - lastStatus.index * 0.001;
    if(timeSinceLastMeasurement == 0.0)
        return lastOutput;
    
    const double curSpeed = (status.states[base::actuators::WHEEL4_FRONT_LEFT].positionExtern - status.states[base::actuators::WHEEL4_FRONT_LEFT].positionExtern 
                            + status.states[base::actuators::WHEEL4_REAR_LEFT].positionExtern - status.states[base::actuators::WHEEL4_REAR_LEFT].positionExtern)
                            / 2.0 / timeSinceLastMeasurement;
    

    const double output = leftPid.update(curSpeed, wantedSpeed, timeSinceLastMeasurement);
    lastOutput = output;
    
    return output;
}

double SoftTurnController::rightController(double wantedSpeed, const base::actuators::Status &status)
{
    static double lastOutput = 0.0;
    const double timeSinceLastMeasurement = status.index - lastStatus.index * 0.001;
    if(timeSinceLastMeasurement == 0.0)
        return lastOutput;
    
    const double curSpeed = (status.states[base::actuators::WHEEL4_FRONT_RIGHT].positionExtern - status.states[base::actuators::WHEEL4_FRONT_RIGHT].positionExtern 
                            + status.states[base::actuators::WHEEL4_REAR_RIGHT].positionExtern - status.states[base::actuators::WHEEL4_REAR_RIGHT].positionExtern) 
                            / 2.0 / timeSinceLastMeasurement;
    

    const double output = rightPid.update(curSpeed, wantedSpeed, timeSinceLastMeasurement);
    lastOutput = output;
    
    return output;    
}

void SoftTurnController::updateHook()
{
    SoftTurnControllerBase::updateHook();
    
    double pointTurnLimit = 0.1;
    double stepLimit = 0.01;

    static double lastSpeed = 0;
    static base::Time changeTime;

    base::actuators::Status status;
    if(_status.readNewest(status, true) == RTT::NoData)
	return;

    // This is the user's command
    base::commands::Motion2D cmd_in;
    if (_motion_command.readNewest(cmd_in, true) == RTT::NoData)
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }
        
    //let's allways turn
    if(true || fabs(cmd_in.rotation) > pointTurnLimit)
    {
        //hack
        cmd_in.rotation = 1.0;
        cmd_in.rotation = turnSpeed * fabs(cmd_in.rotation) / cmd_in.rotation;

	static bool switchState = false;
	for(int i = 0; i < 4; i++)
	{
	    const double diff = fabs(startStatus.states[i].positionExtern - status.states[i].positionExtern);
	    //we assume zero slip
	    if(diff * _wheel_radius.get()  > _turnVariance.get() )
	    {
		//we moved the maximum variance in one direction, reverse
		switchState = true;
		break;
	    }
	}
	
	if(!switchState)
	{
        switch(state)
        {
            case FORWARD:
                cmd_in.translation = translationalSpeed;		
                break;
                
            case BACKWARD:
                cmd_in.translation = -translationalSpeed;		
                break;
        }
	}
	else 
	{
	    if(changeTime.isNull())
		changeTime = base::Time::now();

	    if(base::Time::now() - changeTime > base::Time::fromSeconds(0.5) )
	    {
		startStatus = status;
		if(state == FORWARD)
		    state = BACKWARD;
		else
		    state = FORWARD;

		std::cout << "Switched State" << std::endl;
		changeTime = base::Time();
		switchState = false;
	    }
	}
    }

    double newSpeed = 0;
    if(cmd_in.translation < 0)
    {
	newSpeed = lastSpeed - stepLimit;
	if(newSpeed < cmd_in.translation)
	    newSpeed = cmd_in.translation;
    }
    else
    {
	newSpeed = lastSpeed + stepLimit;
	if(newSpeed > cmd_in.translation)
	    newSpeed = cmd_in.translation;
    }

    lastSpeed = newSpeed;
    cmd_in.translation = newSpeed;

    std::cout << "cmd_in.translation " << cmd_in.translation << "cmd_in.rotation " << cmd_in.rotation << std::endl; 
    double fwd_velocity = cmd_in.translation / _wheel_radius.get();
    double differential = cmd_in.rotation * _track_width.get() / _wheel_radius.get();
    
    double leftSpeed;
    double rightSpeed;
    
    m_cmd.mode[0] = m_cmd.mode[1] =
        m_cmd.mode[2] = m_cmd.mode[3] = base::actuators::DM_SPEED;

    
    m_cmd.target[base::actuators::WHEEL4_FRONT_LEFT]  = fwd_velocity - differential;
    m_cmd.target[base::actuators::WHEEL4_REAR_LEFT]   = fwd_velocity - differential;
    m_cmd.target[base::actuators::WHEEL4_FRONT_RIGHT] = fwd_velocity + differential;
    m_cmd.target[base::actuators::WHEEL4_REAR_RIGHT]  = fwd_velocity + differential;
    m_cmd.time = base::Time::now();

    _simple_command.write(m_cmd);
}
// void SoftTurnController::errorHook()
// {
//     SoftTurnControllerBase::errorHook();
// }
// void SoftTurnController::stopHook()
// {
//     SoftTurnControllerBase::stopHook();
// }
// void SoftTurnController::cleanupHook()
// {
//     SoftTurnControllerBase::cleanupHook();
// }

