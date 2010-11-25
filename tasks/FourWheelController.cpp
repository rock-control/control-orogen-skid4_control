/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FourWheelController.hpp"

using namespace skid4_control;

FourWheelController::FourWheelController(std::string const& name, TaskCore::TaskState initial_state)
    : FourWheelControllerBase(name, initial_state)
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FourWheelController.hpp for more detailed
// documentation about them.

// bool FourWheelController::configureHook()
// {
//     if (! FourWheelControllerBase::configureHook())
//         return false;
//     return true;
// }
// bool FourWheelController::startHook()
// {
//     if (! FourWheelControllerBase::startHook())
//         return false;
// 
//     return true;
// }

void FourWheelController::updateHook()
{
    controldev::FourWheelCommand oInputCmd;
    if (_four_wheel_command.read(oInputCmd) == RTT::NoData)
    {
        // No data on input, send last command on output
        _simple_command.write(m_cmd);
        return;
    }

    for (int i = 0; i < 4; ++i)
    {
        m_cmd.mode[i]   = oInputCmd.mode[i];
        m_cmd.target[i] = oInputCmd.target[i];
    }

    _simple_command.write(m_cmd);
}

// void FourWheelController::errorHook()
// {
// }
void FourWheelController::stopHook()
{
    for(int i = 0; i < 4 ; i++)
    {
        m_cmd.mode[i] = base::actuators::DM_PWM;
        m_cmd.target[i] = 0;
    }
    m_cmd.time = base::Time::now();
    _simple_command.write(m_cmd);
}
// void FourWheelController::cleanupHook()
// {
// }

