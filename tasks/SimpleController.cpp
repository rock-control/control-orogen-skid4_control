/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SimpleController.hpp"

using namespace skid4_control;

SimpleController::SimpleController(std::string const& name, TaskCore::TaskState initial_state)
    : SimpleControllerBase(name, initial_state)
{
}

void SimpleController::updateHook()
{
    SimpleControllerBase::updateHook();

    // This is the user's command
    base::MotionCommand2D cmd_in;
    if (_motion_command.read(cmd_in) == RTT::NoData)
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }

    // The output of this controller is a speed command.
    m_cmd.mode[0] = m_cmd.mode[1] =
        m_cmd.mode[2] = m_cmd.mode[3] = base::actuators::DM_SPEED;

    double fwd_velocity = cmd_in.translation / _wheel_radius.get();
    double differential = cmd_in.rotation * _track_width.get() / _wheel_radius.get();
    m_cmd.target[base::actuators::WHEEL4_FRONT_LEFT]  = fwd_velocity - differential;
    m_cmd.target[base::actuators::WHEEL4_REAR_LEFT]   = fwd_velocity - differential;
    m_cmd.target[base::actuators::WHEEL4_FRONT_RIGHT] = fwd_velocity + differential;
    m_cmd.target[base::actuators::WHEEL4_REAR_RIGHT]  = fwd_velocity + differential;
    m_cmd.time = base::Time::now();

    _simple_command.write(m_cmd);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SimpleController.hpp for more detailed
// documentation about them.

// bool SimpleController::configureHook()
// {
//     if (! SimpleControllerBase::configureHook())
//         return false;
//     return true;
// }
// bool SimpleController::startHook()
// {
//     if (! SimpleControllerBase::startHook())
//         return false;
//     return true;
// }
// void SimpleController::errorHook()
// {
//     SimpleControllerBase::errorHook();
// }
// void SimpleController::stopHook()
// {
//     SimpleControllerBase::stopHook();
// }
// void SimpleController::cleanupHook()
// {
//     SimpleControllerBase::cleanupHook();
// }

