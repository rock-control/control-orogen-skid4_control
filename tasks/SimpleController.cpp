/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SimpleController.hpp"
#include <base/commands/Motion2D.hpp>

using namespace skid4_control;

SimpleController::SimpleController(std::string const& name)
    : SimpleControllerBase(name)
{
}

SimpleController::SimpleController(std::string const& name, RTT::ExecutionEngine* engine)
    : SimpleControllerBase(name, engine)
{
}


bool SimpleController::configureHook()
{
    if(!SimpleControllerBase::configureHook())
        return false;
    
    m_radius = _wheel_radius.get();
    m_trackWidth = _track_width.get();
    if (m_radius <= 0 || m_trackWidth <= 0)
    {
        RTT::log(RTT::Error) << "wrong radius and/or track_width parameters" << RTT::endlog();
        return false;
    }
    
    return true;    
}

void SimpleController::updateHook()
{
    SimpleControllerBase::updateHook();

    // This is the user's command
    base::commands::Motion2D cmd_in;
    if (_motion_command.readNewest(cmd_in) == RTT::NoData)
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }


    double fwd_velocity = cmd_in.translation / m_radius;
    double differential = cmd_in.rotation * m_trackWidth / m_radius;

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
