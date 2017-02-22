/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Controller.hpp"

using namespace skid4_control;

Controller::Controller(std::string const& name, TaskCore::TaskState initial_state)
    : ControllerBase(name, initial_state)
{
}

Controller::Controller(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ControllerBase(name, engine, initial_state)
{
}

Controller::~Controller()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Controller.hpp for more detailed
// documentation about them.

bool Controller::configureHook()
{
    if (! ControllerBase::configureHook())
        return false;
    
    
    m_LeftWheelNames = _left_wheel_names.get();
    m_RightWheelNames = _right_wheel_names.get();
    
    m_jointCmd.resize(m_LeftWheelNames.size() + m_RightWheelNames.size());
    
    size_t curIndex = 0;
    for(std::vector<std::string>::iterator it = m_LeftWheelNames.begin(); it != m_LeftWheelNames.end() ; it++)
    {
        m_jointCmd.names[curIndex] = *it;
        m_leftIndexes.push_back(curIndex);
        curIndex++;
    }
    
    for(std::vector<std::string>::iterator it = m_RightWheelNames.begin(); it != m_RightWheelNames.end() ; it++)
    {
        m_jointCmd.names[curIndex] = *it;
        m_rightIndexes.push_back(curIndex);
        curIndex++;        
    }

    maxTorque = _max_torque.get();
    
    if(maxTorque < 0)
        maxTorque = base::unset<float>();
    
    return true;
}

bool Controller::startHook()
{
    if (! ControllerBase::startHook())
        return false;

    return true;
}

// void Controller::updateHook()
// {
//     ControllerBase::updateHook();
// }
// void Controller::errorHook()
// {
//     ControllerBase::errorHook();
// }
// void Controller::stopHook()
// {
//     ControllerBase::stopHook();
// }
// void Controller::cleanupHook()
// {
//     ControllerBase::cleanupHook();
// }

