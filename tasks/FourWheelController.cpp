#include "FourWheelController.hpp"

#include <rtt/NonPeriodicActivity.hpp>

using namespace control;


RTT::NonPeriodicActivity* FourWheelController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


FourWheelController::FourWheelController(std::string const& name, TaskCore::TaskState initial_state)
    : FourWheelControllerBase(name, initial_state)
{
    for(int i = 0; i < 4 ; i++)
    {
        wmcmd.mode[i] = hbridge::DM_PWM;
        wmcmd.target[i] = 0;
    }
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FourWheelController.hpp for more detailed
// documentation about them.

// bool FourWheelController::configureHook()
// {
//     return true;
// }
// bool FourWheelController::startHook()
// {
//     return true;
// }

void FourWheelController::updateHook()
{
        controldev::RawCommand oInputCmd;
        float sliderValues[7];
        if (!_raw_cmd_in.read(oInputCmd))
        {
            // No data on input, send last command on output
            _cmd_out.write(wmcmd);
            return;
        }



	// Writing out the message
	_cmd_out.write(wmcmd);
}

// void FourWheelController::errorHook()
// {
// }
// void FourWheelController::stopHook()
// {
// }
// void FourWheelController::cleanupHook()
// {
// }

