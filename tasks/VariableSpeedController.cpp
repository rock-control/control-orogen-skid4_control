#include "VariableSpeedController.hpp"
#include "../Robot.hpp"
#include<iostream>
//#include<math>

/*
   Extra Variables Used  :
   calib_flag 	 - Go into calibration mode
   fwd_flag 	 - Forward Direction calibration/ Reverse Direction calibration
   last_position[4] - Keep Track of the last position during calibration
   n 		 - Used while mapping the absolute angle into relative angle of range 0 - PI/5
   still_motor 	 - Used to check if the wheel is in same position for longer time	
   */

using namespace std;

using namespace control;
using namespace RTT;

VariableSpeedController::VariableSpeedController(std::string const& name, TaskCore::TaskState initial_state)
    : VariableSpeedControllerBase(name, initial_state)
{	
    fwd_flag = 1;;
    calib_flag =1;
    for (int i = 0; i < 4; ++i)
    {
        still_motor[i] = 0;			//Initializing all the variables
        last_pos[i] = 0;
    }
}




bool VariableSpeedController::drive(double translation, double rotation, int duration)
{
    return true;
}
bool VariableSpeedController::isdriveCompleted(double translation, double rotation, int duration)
{
    return true;
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

void VariableSpeedController::updateHook()
{
    // This is the user's command
    controldev::MotionCommand cmd_in;
    if (! _motion_command.read(cmd_in))
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }

    // This is the hbridge status
    hbridge::Status status;
    if (! _status.read(status))
    {
        return;
    }


	hbridge::SimpleCommand cmd_out;


    if(calib_flag) // Entering Calibration
    {
        cmd_out.mode[0] = cmd_out.mode[1] =                             //Setting PWM mode
            cmd_out.mode[2] = cmd_out.mode[3] = hbridge::DM_PWM;

        //cout<<"Entered Calibration "<<endl;

        float test_pwm;

        if(fwd_flag) 			// doing calibration in the forward direction
            test_pwm = 0.13;
        else
            test_pwm = -0.13;

        bool reached_maximum = true;
        for (int i = 0; i < 4; i++)
        {
            if (still_motor[i] < 300)
            {
                cmd_out.target[i] = test_pwm; //Slow PWM in the forward direction         		
                if (fabs(last_pos[i] - status.states[i].position) < 0.001)
                    still_motor[i]++;
                else
                    still_motor[i] = 0;

                reached_maximum = false;
                last_pos[i] = status.states[i].position;
            }
            else
                cmd_out.target[i] = 0;			
        }
        _simple_command.write(cmd_out);	



        if (reached_maximum)
        {
            if (fwd_flag)
            {	
                cout<<"Forward Calibration Over"<<endl;
                for(int i=0;i<4;i++)	
                {
                    init_pos[i]= status.states[i].position; // store initial calibration data
                    still_motor[i] =0; // Re-Initialize to Zero for the reverse direction calibration
                }
                fwd_flag = 0;
                reached_maximum = false;
                cout<<"Maximum Reset ";
            }
            else
            {	
                for(int i=0;i<4;i++)
                    final_pos[i]= status.states[i].position;// store final calibration data

                for(int i =0;i<4;++i)
                    mid_pos[i] = (init_pos[i] + final_pos[i])/2;    // Mid position  for each wheel

                calib_flag = 0;
                cout<<"Calibration Done - Use Joystick to Control"<<endl;
            }
        }
    }
    // Calibration Loop over 
    else // Normal Motion with Variable speed 
    {	
        cmd_out.mode[0] = cmd_out.mode[1] =
            cmd_out.mode[2] = cmd_out.mode[3] = hbridge::DM_POSITION; // Changing to Speed Mode

        /*
           Need to Convert the absolute angle from the status into range [ 0 - 36 deg(pi/5)].
           Initially n = 0;
           whenever the angle becomes 36 , increase 'n' and subtract so that the present angle is 0. ( Due to even-ness of the cosine function)
           Since the Mid position ..ie Zero corresponds to PI/5 , divide the translational velocity by cos(PI/5 - angle)
           Add/subtract the differential to the corr wheel target.

*/
        //Applying the variable speed to individual motors
 /*
        for(int i=0;i<4;i++)
        {	
            angle[i] = fmod(fabs(status.states[i].position - mid_pos[i]),(M_PI/2.5));
            fwd_velocity[i] = cmd_in.translation;	
        } 
        double differential = cmd_in.rotation * robot::ROTATION_RADIUS;
	/*
        cmd_out.target[MOTOR_FRONT_LEFT]  = (fwd_velocity[MOTOR_FRONT_LEFT] - differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[0]));
        cmd_out.target[MOTOR_REAR_LEFT]   = (fwd_velocity[MOTOR_REAR_LEFT] - differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[1]));
        cmd_out.target[MOTOR_FRONT_RIGHT] = (fwd_velocity[MOTOR_FRONT_RIGHT] + differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[2]));
        cmd_out.target[MOTOR_REAR_RIGHT]  = (fwd_velocity[MOTOR_REAR_RIGHT] + differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[3]));
	
	
        cmd_out.target[0]  = (fwd_velocity[0] - differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[0]));
        cmd_out.target[1]  = (fwd_velocity[1] + differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[1]));
        cmd_out.target[2]  = (fwd_velocity[2] + differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[2]));
        cmd_out.target[3]  = (fwd_velocity[3] - differential)/ (robot::WHEEL_RADIUS * cos((M_PI/5) -angle[3]));
*/
	cmd_out.target[0]  = mid_pos[0] ;
	cmd_out.target[1]  = mid_pos[1] + M_PI/5 ;
	cmd_out.target[2]  = mid_pos[2] ;
	cmd_out.target[3]  = mid_pos[3] + M_PI/5;
        _simple_command.write(cmd_out);	

    }		
}
// void VariableSpeedController::errorHook()
// {
// }
// void VariableSpeedController::stopHook()
// {
// }
// void VariableSpeedController::cleanupHook()
// {
// }

