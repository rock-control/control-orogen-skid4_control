#include "FourWheelController.hpp"

#include <rtt/NonPeriodicActivity.hpp>

#define	SLIDER_MAX 252			/*   */
#define	SLIDER_MIN 3			/*   */
#define	VEL_MAX_PWM 1.0			/*   */
#define	VEL_MAX_PID 7.0			/*   */

#define	SLIDER_MASTER_VEL_LEFT 1			/*   */
#define	SLIDER_VEL_LEFT_FRONT 2			/*   */
#define	SLIDER_VEL_LEFT_REAR 3		/*   */

#define	SLIDER_MASTER_VEL_RIGHT 6			/*   */
#define	SLIDER_VEL_RIGHT_REAR 4			/*   */
#define	SLIDER_VEL_RIGHT_FRONT 5			/*   */

#define	SLIDER_ICR 0			/*   */


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

        for (int i = 0; i < 7; i++)
        {
                // Reads the slider value from MIN to MAX and scales it to 0 to 1
                sliderValues[i] = (( oInputCmd.sliderValues[i]- SLIDER_MIN) / (SLIDER_MAX-SLIDER_MIN));

                // Rounds the data outside MIN and MAX
                if (sliderValues[i] > 1.0)
                        sliderValues[i] = 1.0;
                else if (sliderValues[i] < 0.0)
                        sliderValues[i] = 0.0;
        }

        if (oInputCmd.devices & controldev::DAI_SliderBox)
        {

                /* ON state */	
                if ( oInputCmd.sliderButtons & (1 << 3) )/* Button 4 */
                {
                        float vel_max;   // Sets the velocity to be scaled in different modes

                        /* Sets the control mode for each motor */
                        for (int i=0;i<4;i++)
                        {
                                // Sets the mode based on status of Button 3
                                if (oInputCmd.sliderButtons & (1 << 2))  /* Button 3 */
                                {
                                        /* PWM Mode */
                                        wmcmd.mode[i] = hbridge::DM_PWM;
                                        vel_max = VEL_MAX_PWM;
                                }
                                else
                                {
                                        /* PID Mode */
                                        wmcmd.mode[i] = hbridge::DM_SPEED;
                                        vel_max = VEL_MAX_PID;
                                }
                        }

                        /* Sets the driving mode and calulates the individual velocities*/

                        /* Individual wheel and Master wheel set drive */
                        if (oInputCmd.sliderButtons & (1 << 1))   /* Button 2 */
                        {
                                
                                wmcmd.target[3] = 2  * (sliderValues[SLIDER_MASTER_VEL_LEFT]  - 0.5) 
                                        * sliderValues[SLIDER_VEL_LEFT_FRONT]
                                        * vel_max;
                                wmcmd.target[0] = 2  * (sliderValues[SLIDER_MASTER_VEL_LEFT]  - 0.5)
                                        * sliderValues[SLIDER_VEL_LEFT_REAR]
                                        * vel_max;
                                wmcmd.target[1] = 2  * (sliderValues[SLIDER_MASTER_VEL_RIGHT] - 0.5)
                                        * sliderValues[SLIDER_VEL_RIGHT_REAR]
                                        * vel_max;
                                wmcmd.target[2] = 2  * (sliderValues[SLIDER_MASTER_VEL_RIGHT] - 0.5)
                                        * sliderValues[SLIDER_VEL_RIGHT_FRONT]
                                        * vel_max;
                        }
                        /* ICR (Instantaneous Center of Rotation) drive */
                        else
                        {
                                // If slider position is on the left half ... turn left
                                if ( (sliderValues[SLIDER_ICR] ) <= 0.5 )					
                                {
                                        // Master Vel * ICR position * max vel
                                        wmcmd.target[3] = 2 * (sliderValues[SLIDER_MASTER_VEL_LEFT] - 0.5) *
                                                4 * (sliderValues[SLIDER_ICR] - 0.25)
                                                * vel_max;
                                        wmcmd.target[0] = 2 * (sliderValues[SLIDER_MASTER_VEL_LEFT] - 0.5) *
                                                4 * (sliderValues[SLIDER_ICR] - 0.25)
                                                * vel_max;

                                        // Master vel * max vel
                                        wmcmd.target[1] = 2 * (sliderValues[SLIDER_MASTER_VEL_RIGHT] - 0.5) *
                                                vel_max;
                                        wmcmd.target[2] = 2 * (sliderValues[SLIDER_MASTER_VEL_RIGHT] - 0.5) *
                                                vel_max;
                                }
                                // If slider position is on the right half ... turn right
                                else
                                {
                                        // Master vel * max vel					
                                        wmcmd.target[3] = 2 * (sliderValues[SLIDER_MASTER_VEL_LEFT] - 0.5) *
                                                vel_max;
                                        wmcmd.target[0] = 2 * (sliderValues[SLIDER_MASTER_VEL_LEFT] - 0.5) *
                                                vel_max;

                                        // Master vel + ICR position * max vel
                                        wmcmd.target[1] = 2 * (sliderValues[SLIDER_MASTER_VEL_RIGHT] - 0.5) *
                                                4 * (-sliderValues[SLIDER_ICR] + 0.75)
                                                * vel_max;
                                        wmcmd.target[2] = 2 * (sliderValues[SLIDER_MASTER_VEL_RIGHT] - 0.5) *
                                                4 * (-sliderValues[SLIDER_ICR] + 0.75)
                                                * vel_max;
                                }
                        }
                } 
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

