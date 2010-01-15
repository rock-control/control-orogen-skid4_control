#include "TwoPhaseController.hpp"
#include "../Robot.hpp"
#include<iostream>

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

TwoPhaseController::TwoPhaseController(std::string const& name, TaskCore::TaskState initial_state)
    : TwoPhaseControllerBase(name, initial_state)
{	
    fwd_flag = 1;;
    calib_flag =1;
    for (int i = 0; i < 4; ++i)
    {
        still_motor[i] = 0;			//Initializing all the variables
        last_pos[i] = 0;
	touchdown[i] = 0;
	v_1[i] = 0;
	time_sync[i] = 0;
	last_touchdown[i] = 0;
	last_state[i]= 0;
    }
}




bool TwoPhaseController::drive(double translation, double rotation, int duration)
{
    return true;
}
bool TwoPhaseController::isdriveCompleted(double translation, double rotation, int duration)
{
    return true;
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

void TwoPhaseController::updateHook()
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

   
     controller::TwoPhaseState state;

    hbridge::SimpleCommand cmd_out;


    if(calib_flag) // Entering Calibration
    {

        cmd_out.mode[0] = cmd_out.mode[1] =                             //Setting PWM mode
            cmd_out.mode[2] = cmd_out.mode[3] = hbridge::DM_PWM;

        //cout<<"Entered Calibration "<<endl;

        float test_pwm;

        if(fwd_flag) 			// doing calibration in the forward direction
            test_pwm = 0.1;
        else
            test_pwm = -0.1;

        bool reached_maximum = true;
        for (int i = 0; i < 4; i++)
        {
            state.motors[i].touchdown_time = 0;
            state.motors[i].phase = -1;
            state.motors[i].last_phase = -1;
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
        _internal_state.write(state);



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
		{
                    mid_pos[i] = (init_pos[i] + final_pos[i])/2;    // Mid position  for each wheel
                    cout<<endl<<i<<". initital position = " <<init_pos[i]<< "     Final Position  = "<< final_pos[i] << "   Middle position = "<<mid_pos[i];
		}	

                calib_flag = 0;
                cout<<endl<<"Calibration Done - Use Joystick to Control"<<endl;
            }
        }
    }
   // Calibration Loop over 
    else // Normal Motion with Variable speed 
    {
	
	cmd_out.mode[0] = cmd_out.mode[1] =
	    cmd_out.mode[2] = cmd_out.mode[3] = hbridge::DM_SPEED; // Changing to Speed Mode.
	if(cmd_in.translation)
	{

	      
		double differential = cmd_in.rotation * robot::ROTATION_RADIUS;
	
		//double period = 2*M_PI/(5*cmd_in.translation/robot::WHEEL_RADIUS); // Defining the values for the parameters

		left_tick = 2*M_PI/(5*(cmd_in.translation - differential)/robot::WHEEL_RADIUS);

		right_tick = 2*M_PI/(5*(cmd_in.translation + differential)/robot::WHEEL_RADIUS);

		bool first_flag; // used to calculate the constant velocity for the first time in the sync mode.

		for (int i=0;i<4;i++)
		{
			double p = fabs(fmod((status.states[i].position - mid_pos[i]),(M_PI/5)));
		
			//double period = 0;
		

			if (p <= M_PI/10) // sync mode . The gnd/sync mode is decided by this condition
			{
				uint64_t period;
		
				if(i == 0 || i == 3 ) // Check for the left side
					period = left_tick;	
				else if(i == 1 || i ==2 ) // Check for the right side
					period = right_tick;
		
				uint64_t  t_touchdown = (int((status.index - last_touchdown[i])/period)* period)  + period;
			
				if (i == 0 || i == 2)
					t_touchdown += period / 2;
				if(i == 0 || i == 3 )
					differential = - differential;
		
					//cout<<"Sync Mode";
				touchdown[i]  = t_touchdown;
			
				if(last_state[i] == 1/*PHASE_GROUND*/) // Finding the start of the sync phase (t_s)
				{
					time_sync[i] = status.index;
					first_flag = 1;
				}

				double remaining_time = t_touchdown - status.index;
				double remaining_pos  = p - M_PI / 10;
	
				if (remaining_pos > M_PI/15  && remaining_time < period/6) // Skip the sync phase when the time is less and the position is more
				     remaining_time += period;
	
				if(remaining_time <= 4*period/10) // Constant velocity during the time ( t_s to (t_g - period/10)) 
				{	
					if(first_flag)//Constant velocity is decided by the position while entering the sync mode
					{
						cmd_out.target[i] = find_speed_1(status.states[i].position - mid_pos[i] , cmd_in.translation/robot::WHEEL_RADIUS ,(touchdown[i] - time_sync[i])); // constant velocity
						v_1[i] = cmd_out.target[i]; // Storing the constant  velocity found for finding the v(t) during the rest of the velocity profile
						first_flag = 0;
					}
					else
						cmd_out.target[i] = v_1[i]; // Outputting the previous remembered constant velocity 
				
				}																		
				else if(remaining_time > 4*period/10) // Sloping velocity during the time ((t_g - period/10) to t_g)
					cmd_out.target[i] = find_speed_2(v_1[i], (touchdown[i] - time_sync[i]), cmd_in.translation/robot::WHEEL_RADIUS ,(status.index - t_touchdown));

				last_state[i] = 0/*PHASE_SYNC*/;
		
	    		        state.motors[i].phase = 0;	
			        state.motors[i].touchdown_time = t_touchdown;
	
			} 
			else // Ground Mode
			{


				if(last_state[i] == 0/*PHASE_SYNC*/)
				    last_touchdown[i] = touchdown[i];
	
		
				cmd_out.target[i] = (cmd_in.translation + differential )/robot::WHEEL_RADIUS;  // the differential is set accd to the side at the start 			
				v_1[i] = cmd_out.target[i];
		
				last_state[i] = 1/*PHASE_GROUND*/;
		                state.motors[i].phase = 1;
			        state.motors[i].touchdown_time = last_touchdown[i];

			}
			// to make sure that cmd_out.target[i] is between [- v_ground * 1.2, - v_ground * 0.8] U [v_ground * 0.8, v_ground * 1.2]
		
	
			if(fabs(cmd_out.target[i]< (0.4*(cmd_in.translation + differential)/robot::WHEEL_RADIUS)))	
					cmd_out.target[i] = 0.4*((cmd_in.translation + differential)/robot::WHEEL_RADIUS);
	
			else if(fabs(cmd_out.target[i] > (1.8*(cmd_in.translation + differential)/robot::WHEEL_RADIUS)))	
					cmd_out.target[i] = 1.8*((cmd_in.translation + differential)/robot::WHEEL_RADIUS);
		
			state.motors[i].last_phase = last_state[i];
	  	
			
		}	

	}
	else
	{
		for (int i=0;i<4;i++)
		{
			cmd_out.target[i] = 0;
		        state.motors[i].phase = -1;	
			state.motors[i].touchdown_time = last_touchdown[i];
			state.motors[i].last_phase = -1;
		}
	}
	_internal_state.write(state);
	_simple_command.write(cmd_out);	


    }		
}
	/*
double find_speed_1(p_abs , v_ground , period_rem) // period_rem = t_g - t_s . Depends  on (t_g - t_s) and v_ground and p_abs
{
	p_ground = ceil((p_abs - M_PI/10)/(2*M_PI/5))*2*M_PI/5 + M_PI/10 ;  // As governed by the equations calculated
	return ((p_ground - v_ground*period_rem/20)/(9*period_rem/20));
}


double find_speed_2 (v_1 , period_rem,v_ground,time)
{
	return ( v_1 + (((v_ground - v_1)*(time - (4*period_rem/10)))/M_PI/10) ) ; // As governed by the equations calculated
}

/*
Velocity Profile in the Sync Phase :
	____________
	           |\<--find_speed_2
	<---v_1--->| \
        l-> find_speed_1

find_speed_1 :
	     is used to find the constant part in the velocity profie of the sync phase. 
	     Depends on the P_g and the v_ground.

find_speed_2 :
	     is used to find the v(t) at the sloping part in velocity profile of the sync phase.
	     Depends on the constant velocity found in find_speed_1 and the time. 

*/


//Variables :
//left_tick , right_tick,touchdown[4],last_state[4] ,v_1[4], time_sync[4]

// Phase_sync = 0, Phase_gnd = 1; 
// void TwoPhaseController::errorHook()
// {
// }
// void TwoPhaseController::stopHook()
// {
// }
// void TwoPhaseController::cleanupHook()
// {
// }

