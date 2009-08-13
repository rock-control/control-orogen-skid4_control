#include "VariableSpeedController.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace control;


RTT::NonPeriodicActivity* VariableSpeedController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


VariableSpeedController::VariableSpeedController(std::string const& name, TaskCore::TaskState initial_state)
    : VariableSpeedControllerBase(name, initial_state)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VariableSpeedController.hpp for more detailed
// documentation about them.

// bool VariableSpeedController::configureHook()
// {
//     return true;
// }
// bool VariableSpeedController::startHook()
// {
//     return true;
// }

// void VariableSpeedController::updateHook()
// {
// }

// void VariableSpeedController::errorHook()
// {
// }
// void VariableSpeedController::stopHook()
// {
// }
// void VariableSpeedController::cleanupHook()
// {
// }

