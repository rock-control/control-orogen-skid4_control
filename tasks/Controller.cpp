#include "Controller.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace control;


RTT::NonPeriodicActivity* Controller::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


Controller::Controller(std::string const& name, TaskCore::TaskState initial_state)
    : ControllerBase(name, initial_state)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Controller.hpp for more detailed
// documentation about them.

// bool Controller::configureHook()
// {
//     return true;
// }
// bool Controller::startHook()
// {
//     return true;
// }

// void Controller::updateHook()
// {
// }

// void Controller::errorHook()
// {
// }
// void Controller::stopHook()
// {
// }
// void Controller::cleanupHook()
// {
// }

