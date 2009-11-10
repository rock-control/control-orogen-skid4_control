#include "PIVController.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace control;


RTT::NonPeriodicActivity* PIVController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


    PIVController::PIVController(std::string const& name, TaskCore::TaskState initial_state)
: PIVControllerBase(name, initial_state)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PIVController.hpp for more detailed
// documentation about them.

// bool PIVController::configureHook()
// {
//     return true;
// }
// bool PIVController::startHook()
// {
//     return true;
// }

void PIVController::updateHook()
{
    // This is the hbridge status
//    hbridge::Status status;
//    if (! _status.read(status))
//    {
//	return;
//    }

    for (int i=0;i<4;i++)
    {
	wmcmd.mode[i] 	= hbridge::DM_SPEED;
	wmcmd.target[i] = 1.5;
    }

    // Writing out the message
    _cmd_out.write(wmcmd);

}

// void PIVController::errorHook()
// {
// }
// void PIVController::stopHook()
// {
// }
// void PIVController::cleanupHook()
// {
// }

