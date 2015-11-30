/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SimpleController.hpp"
#include <base/commands/Motion2D.hpp>
#include <base/actuators/vehicles.h>

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
    
    std::vector<std::string> leftNames(_left_wheel_names.get());
    std::vector<std::string> rightNames(_right_wheel_names.get());
    
    m_jointCmd.resize(leftNames.size() + rightNames.size());
    
    size_t curIndex = 0;
    for(std::vector<std::string>::iterator it = leftNames.begin(); it != leftNames.end() ; it++)
    {
        m_jointCmd.names[curIndex] = *it;
        m_leftIndexes.push_back(curIndex);
        curIndex++;
    }
    
    for(std::vector<std::string>::iterator it = rightNames.begin(); it != rightNames.end() ; it++)
    {
        m_jointCmd.names[curIndex] = *it;
        m_rightIndexes.push_back(curIndex);
        curIndex++;        
    }

    // The output of this controller is a speed command.
    m_cmd.mode[0] = m_cmd.mode[1] = m_cmd.mode[2] = m_cmd.mode[3] = base::actuators::DM_SPEED;
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
    m_cmd.target[base::actuators::WHEEL4_FRONT_LEFT]  = fwd_velocity - differential;
    m_cmd.target[base::actuators::WHEEL4_REAR_LEFT]   = fwd_velocity - differential;
    m_cmd.target[base::actuators::WHEEL4_FRONT_RIGHT] = fwd_velocity + differential;
    m_cmd.target[base::actuators::WHEEL4_REAR_RIGHT]  = fwd_velocity + differential;
    m_cmd.time = base::Time::now();

    _simple_command.write(m_cmd);

    for(std::vector<size_t>::const_iterator it = m_leftIndexes.begin(); it != m_leftIndexes.end();it++)
    {
        m_jointCmd.elements[*it].speed = fwd_velocity - differential;
    }
    for(std::vector<size_t>::const_iterator it = m_rightIndexes.begin(); it != m_rightIndexes.end();it++)
    {
        m_jointCmd.elements[*it].speed = fwd_velocity + differential;
    }
    
    m_jointCmd.time = m_cmd.time;
    
    _command.write(m_jointCmd);
}
