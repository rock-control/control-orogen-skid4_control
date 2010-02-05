#include "FourWheelController.hpp"

#include <rtt/NonPeriodicActivity.hpp>

using namespace control;


RTT::NonPeriodicActivity* FourWheelController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


FourWheelController::FourWheelController(std::string const& name, TaskCore::TaskState initial_state)
    : FourWheelControllerBase(name, initial_state)
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FourWheelController.hpp for more detailed
// documentation about them.

// bool FourWheelController::configureHook()
// {
//     return true;
// }

bool FourWheelController::startHook()
{
    for(int i = 0; i < 4 ; i++)
    {
        wmcmd.mode[i] = hbridge::DM_PWM;
        wmcmd.target[i] = 0;
    }
    return true;
}

void FourWheelController::updateHook()
{
        controldev::FourWheelCommand oInputCmd;
        if (!_four_wheel_command.read(oInputCmd))
        {
            // No data on input, send last command on output
            _simple_command.write(wmcmd);
            return;
        }

        for (int i = 0; i < 4; ++i)
        {
            hbridge::DRIVE_MODE hbridge_mode = hbridge::DM_UNINITIALIZED;
            if (oInputCmd.mode[i] == controldev::MODE_PWM)
                hbridge_mode = hbridge::DM_PWM;
            else if (oInputCmd.mode[i] == controldev::MODE_SPEED)
                hbridge_mode = hbridge::DM_SPEED;

            wmcmd.mode[i] = hbridge_mode;
            wmcmd.target[i] = oInputCmd.target[i];
        }

	_simple_command.write(wmcmd);
}

// void FourWheelController::errorHook()
// {
// }
void FourWheelController::stopHook()
{
    for(int i = 0; i < 4 ; i++)
    {
        wmcmd.mode[i] = hbridge::DM_PWM;
        wmcmd.target[i] = 0;
    }
    _simple_command.write(wmcmd);
}
// void FourWheelController::cleanupHook()
// {
// }

