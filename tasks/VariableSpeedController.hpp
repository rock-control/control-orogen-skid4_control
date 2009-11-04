#ifndef HBRIDGE_SIMPLECONTROLLER_TASK_HPP
#define HBRIDGE_SIMPLECONTROLLER_TASK_HPP

#include "control/VariableSpeedControllerBase.hpp"



namespace control {
    class VariableSpeedController : public VariableSpeedControllerBase
    {
	friend class VariableSpeedControllerBase;
    protected:
    
    
	bool drive(double translation, double rotation, int duration);
	bool isdriveCompleted(double translation, double rotation, int duration);
    
	int calib_flag , fwd_flag ,n;
	double last_pos[4],init_pos[4],final_pos[4],mid_pos[4],one_deg[4],fwd_velocity[4],angle[4];
	int still_motor[4];
	float test_pwm;
	double left_tick , right_tick,touchdown[4],v_1[4],time_sync[4],last_touchdown[4];
	bool last_state[4];

    public:
        VariableSpeedController(std::string const& name = "hbridge::VariableSpeedController", TaskCore::TaskState initial_state = Stopped);
	
	double find_speed_1(double p_abs , double v_ground , double period_rem) // period_rem = t_g - t_s . Depends  on (t_g - t_s) and v_ground and p_abs
	{
		double p_ground = ceil((p_abs - M_PI/10)/(2*M_PI/5))*2*M_PI/5 + M_PI/10 ;  // As governed by the equations calculated
		return ((p_ground - v_ground*period_rem/20)/(9*period_rem/20));
	}
        
	double find_speed_2 (double v_1 , double period_rem,double v_ground,double time)
	{
		return ( v_1 + (((v_ground - v_1)*(time - (4*period_rem/10)))/M_PI/10) ) ; // As governed by the equations calculated
	}


        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        // bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        // bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called. See README.txt for different
         * triggering options.
         *
         * The warning(), error() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeWarning, RunTimeError and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recovered()
         * allows you to go back into the Running state.  In the second case,
         * the errorHook() will be called instead of updateHook() and in the
         * third case the component is stopped and resetError() needs to be
         * called before starting it again.
         *
         */
        void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

