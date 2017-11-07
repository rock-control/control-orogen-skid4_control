/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SafeController.hpp"

using namespace skid4_control;

SafeController::SafeController(std::string const& name)
    : SafeControllerBase(name)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

SafeController::SafeController(std::string const& name, RTT::ExecutionEngine* engine)
    : SafeControllerBase(name, engine)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

SafeController::~SafeController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SafeController.hpp for more detailed
// documentation about them.

bool SafeController::configureHook()
{
    if (! SafeControllerBase::configureHook())
        return false;
    
    m_radius = _wheel_radius.get();
    m_trackRadius = _track_width.get() / 2;
    m_cmd_timeout = base::Timeout(_cmd_timeout.value());
    if (m_radius <= 0 || m_trackRadius <= 0)
    {
        RTT::log(RTT::Error) << "wrong radius and/or track_width parameters" << RTT::endlog();
        return false;
    }
    
    return true;    
}
bool SafeController::startHook()
{
    if (! SafeControllerBase::startHook())
        return false;
    
    gotBody2Gravity = false;
    
    _body2imu.registerUpdateCallback([&] (const base::Time &t) {
	base::samples::RigidBodyState body2Imu;
	if(!_body2imu.get(t, body2Imu))
	  return;
	body2Gravity = base::removeYaw(body2Imu.orientation);
	gotBody2Gravity = true;
    } );
    
    return true;
}
void SafeController::updateHook()
{
    SafeControllerBase::updateHook();

    // This is the user's command
    base::commands::Motion2D cmd_in;
    RTT::FlowStatus status = _motion_command.readNewest(cmd_in);
    if (status == RTT::NoData)
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }
    else if(status == RTT::NewData)
        m_cmd_timeout.restart();
    else if(status == RTT::OldData && m_cmd_timeout.elapsed()) // check for input timeout
        return;

    double fwd_velocity = cmd_in.translation / m_radius;
    double differential = cmd_in.rotation * m_trackRadius / m_radius;

    for(std::vector<size_t>::const_iterator it = m_leftIndexes.begin(); it != m_leftIndexes.end();it++)
    {
        m_jointCmd.elements[*it].speed = fwd_velocity - differential;
    }
    for(std::vector<size_t>::const_iterator it = m_rightIndexes.begin(); it != m_rightIndexes.end();it++)
    {
        m_jointCmd.elements[*it].speed = fwd_velocity + differential;
    }
    
    m_jointCmd.time = base::Time::now();
    
    _command.write(m_jointCmd);
}

void SafeController::errorHook()
{
    SafeControllerBase::errorHook();
}
void SafeController::stopHook()
{
    SafeControllerBase::stopHook();
}
void SafeController::cleanupHook()
{
    SafeControllerBase::cleanupHook();
}
