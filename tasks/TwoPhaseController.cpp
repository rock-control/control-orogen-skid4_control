#include "TwoPhaseController.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace control;


RTT::NonPeriodicActivity* TwoPhaseController::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


TwoPhaseController::TwoPhaseController(std::string const& name, TaskCore::TaskState initial_state)
    : TwoPhaseControllerBase(name, initial_state)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TwoPhaseController.hpp for more detailed
// documentation about them.

// bool TwoPhaseController::configureHook()
// {
//     return true;
// }
// bool TwoPhaseController::startHook()
// {
//     return true;
// }

// void TwoPhaseController::updateHook()
// {
// }

// void TwoPhaseController::errorHook()
// {
// }
// void TwoPhaseController::stopHook()
// {
// }
// void TwoPhaseController::cleanupHook()
// {
// }

