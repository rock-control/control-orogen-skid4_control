/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ConstantSpeedController.hpp"

namespace skid4_control {

ConstantSpeedController::ConstantSpeedController(std::string const& name)
    : ConstantSpeedControllerBase(name), speed(0)
{
}

ConstantSpeedController::ConstantSpeedController(std::string const& name, RTT::ExecutionEngine* engine)
    : ConstantSpeedControllerBase(name, engine), speed(0)
{
}

ConstantSpeedController::~ConstantSpeedController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ConstantSpeedController.hpp for more detailed
// documentation about them.

bool ConstantSpeedController::configureHook()
{
    if (! ConstantSpeedControllerBase::configureHook())
        return false;
    
    speed = _speed.get();
    
    if(isnan(speed) || isinf(speed))
	return false;
    
    return true;
}

// bool ConstantSpeedController::startHook()
// {
//     if (! ConstantSpeedControllerBase::startHook())
//         return false;
//     return true;
// }

void ConstantSpeedController::updateHook()
{
    ConstantSpeedControllerBase::updateHook();
    base::MotionCommand2D cmd_in;
    if(_motion_command.readNewest(cmd_in) == RTT::NoData)
    {
	//no input connected, send zero command
	stopMotors();
	return;
    };

    if(cmd_in.translation > 0.2)
    {
	m_cmd.target[0] = _speed;
	m_cmd.target[1] = _speed;
	m_cmd.target[2] = _speed;
	m_cmd.target[3] = _speed;

	m_cmd.mode[0] = base::actuators::DM_SPEED;
	m_cmd.mode[1] = base::actuators::DM_SPEED;
	m_cmd.mode[2] = base::actuators::DM_SPEED;
	m_cmd.mode[3] = base::actuators::DM_SPEED;
	_simple_command.write(m_cmd);    
    } 
    else 
    {
	stopMotors();
    }

}

// void ConstantSpeedController::errorHook()
// {
//     ConstantSpeedControllerBase::errorHook();
// }

void ConstantSpeedController::stopHook()
{
    ConstantSpeedControllerBase::stopHook();
}

// void ConstantSpeedController::cleanupHook()
// {
//     ConstantSpeedControllerBase::cleanupHook();
// }

}
