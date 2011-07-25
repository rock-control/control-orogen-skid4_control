/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Controller.hpp"

using namespace skid4_control;

Controller::Controller(std::string const& name, TaskCore::TaskState initial_state)
    : ControllerBase(name, initial_state)
{
    m_status.resize(4);
    m_cmd.resize(4);
}

Controller::Controller(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ControllerBase(name, engine, initial_state)
{
    m_status.resize(4);
    m_cmd.resize(4);
}

Controller::~Controller()
{
}

void Controller::stopMotors()
{
   for (int i = 0; i < 4 ; i++)
   {
       m_cmd.mode[i] = base::actuators::DM_PWM;
       m_cmd.target[i] = 0;
   }
   
   _simple_command.write(m_cmd);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Controller.hpp for more detailed
// documentation about them.

// bool Controller::configureHook()
// {
//     if (! ControllerBase::configureHook())
//         return false;
//     return true;
// }

bool Controller::startHook()
{
    if (! ControllerBase::startHook())
        return false;

    for (int i = 0; i < 4 ; i++)
    {
        m_cmd.mode[i] = base::actuators::DM_PWM;
        m_cmd.target[i] = 0;
    }
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

