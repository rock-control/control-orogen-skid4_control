#ifndef CONTROL_PIVCONTROLLER_TASK_HPP
#define CONTROL_PIVCONTROLLER_TASK_HPP

#include "control/PIVControllerBase.hpp"
#include "PIV.hpp"
#include "Ramp.hpp"
#include "SimpleIntegrator.hpp"

namespace RTT
{
	class NonPeriodicActivity;
}


namespace control {
	class PIVController : public PIVControllerBase
	{
		friend class PIVControllerBase;
		protected:
		controldev::FourWheelCommand refVel;
		hbridge::SimpleCommand wmcmd;
    		controldev::MotionCommand mcmd;
		motor_controller::Ramp oRamp;
		
	  	// Data members for Controlling	
		motor_controller::PIV oPIV[4];
		double refPos[4];
		double actPos[4], actVel[4];
		int prevIndex, currIndex;
		double prevPos[4];
		double errPos[4];
		bool firstRun;
		bool sync_prev;
                bool calib;
		SimpleIntegrator refVelIntegrator[4];	

		// Data members for calibration
		bool calibrated;
		bool forward;
		double last_pos[4],init_pos[4],final_pos[4],mid_pos[4];
		int still_motor[4];
	

		// Functions	
                bool validInput(controldev::FourWheelCommand const& refVel) const;
		bool calibrate(hbridge::Status status);
		void setSyncRefPos(hbridge::Status status);
		void motionToFourWheelCmd();

		public:
		PIVController(std::string const& name = "control::PIVController", TaskCore::TaskState initial_state = Stopped);

		RTT::NonPeriodicActivity* getNonPeriodicActivity();

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
		bool startHook();

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
		void stopHook();

		/** This hook is called by Orocos when the state machine transitions
		 * from Stopped to PreOperational, requiring the call to configureHook()
		 * before calling start() again.
		 */
		// void cleanupHook();
	};
}

#endif

