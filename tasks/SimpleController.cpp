#include "SimpleController.hpp"
#include <HBridge.hpp>


using namespace control;
using namespace RTT;



SimpleController::SimpleController(std::string const& name, TaskCore::TaskState initial_state)
    : SimpleControllerBase(name, initial_state)
{
}

void SimpleController::updateHook()
{
    // This is the user's command
    controldev::MotionCommand cmd_in;
    if (! _motion_command.read(cmd_in))
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }

    // This is the hbridge status
    // hbridge::Status status;
    // if (! _status.read(status))
    // {
    //     log(Error) 
    //         << "no input status. Did you use a data connection type for the status input port ?"
    //         << endlog();
    //     return error();
    // }

    // Finally, this is the command we have to send to the hbridge at the end
    // of the updateHook()
    hbridge::SimpleCommand cmd_out;
    
    // The output of this controller is a speed command.
    cmd_out.mode[0] = cmd_out.mode[1] =
        cmd_out.mode[2] = cmd_out.mode[3] = hbridge::DM_SPEED;

    double fwd_velocity = cmd_in.translation / ROBOT.WHEEL_RADIUS_EFF;
    double differential = cmd_in.rotation * ROBOT.TRACK / ROBOT.WHEEL_RADIUS_EFF;
    cmd_out.target[hbridge::MOTOR_FRONT_LEFT]  = fwd_velocity - differential;
    cmd_out.target[hbridge::MOTOR_REAR_LEFT]   = fwd_velocity - differential;
    cmd_out.target[hbridge::MOTOR_FRONT_RIGHT] = fwd_velocity + differential;
    cmd_out.target[hbridge::MOTOR_REAR_RIGHT]  = fwd_velocity + differential;

    _simple_command.write(cmd_out);
}

// void SimpleController::errorHook()
// {
// }
// void SimpleController::stopHook()
// {
// }
// void SimpleController::cleanupHook()
// {
// }

