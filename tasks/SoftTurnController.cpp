/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SoftTurnController.hpp"
#include "motor_controller/PID.hpp"
#include <base/commands/Motion2D.hpp>
#include <base/commands/Joints.hpp>

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

double SoftTurnController::getSpeed( const base::samples::Joints &status, const std::vector< std::string > &joints)
{
    double speed = 0;
    const double timeSinceLastMeasurement = (status.time - lastStatus.time).toMilliseconds();
    
    for(std::vector<std::string>::const_iterator it = joints.begin(); it != joints.end();it++)
    {
        if(status[*it].hasSpeed())
        {
            speed += status[*it].speed;
        }
        else
        {
            speed += (status[*it].position - lastStatus[*it].position) / timeSinceLastMeasurement;
        }
    }
    
    speed /= joints.size();
    
    return speed;
}

double SoftTurnController::leftController(double wantedSpeed, const base::samples::Joints &status)
{    
    static double lastOutput = 0.0;
    const double timeSinceLastMeasurement = (status.time - lastStatus.time).toMilliseconds();
    if(timeSinceLastMeasurement == 0.0)
        return lastOutput;
    
    const double curSpeed = getSpeed(status, m_LeftWheelNames);

    const double output = leftPid.update(curSpeed, wantedSpeed, timeSinceLastMeasurement);
    lastOutput = output;
    
    return output;
}

double SoftTurnController::rightController(double wantedSpeed, const base::samples::Joints &status)
{
    static double lastOutput = 0.0;
    const double timeSinceLastMeasurement = (status.time - lastStatus.time).toMilliseconds();
    if(timeSinceLastMeasurement == 0.0)
        return lastOutput;
    
    const double curSpeed = getSpeed(status, m_RightWheelNames);

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

    base::samples::Joints status;
    if(_status_samples.readNewest(status, true) == RTT::NoData)
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
	for(unsigned int i = 0; i < startStatus.size(); i++)
	{
	    const double diff = fabs(startStatus[i].position - status[i].position);
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
    
    m_jointCmd.time = base::Time::now();
    
    for(std::vector<size_t>::const_iterator it = m_rightIndexes.begin(); it != m_rightIndexes.end();it++)
    {
        m_jointCmd[*it].speed = fwd_velocity + differential;
    }

    for(std::vector<size_t>::const_iterator it = m_leftIndexes.begin(); it != m_leftIndexes.end();it++)
    {
        m_jointCmd[*it].speed = fwd_velocity + differential;
    }

    _command.write(m_jointCmd);
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

