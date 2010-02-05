/*
 * =====================================================================================
 *
 *       Filename:  ramp.cpp
 *
 *    Description:  RAMP UP or RAMP DOWN function implementation
 *
 *        Version:  1.0
 *        Created:  02/04/10 10:55:12
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu (), ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */
#include "ramp.h"
/*
 *--------------------------------------------------------------------------------------
 *       Class:  Ramp
 *      Method:  Ramp
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
Ramp::Ramp ()
{
    setDeltaTime(0.0);
    setInitialData(0.0);
    setFinalData(0.0);
    setType(LINEAR);
    active = false;
    bReset = false;
}  /* -----  end of method Ramp::Ramp  (constructor)  ----- */

Ramp::Ramp (double initial_data, double final_data, double delta_time, int type)
{
    setDeltaTime(delta_time);
    setInitialData(initial_data);
    setFinalData(final_data);
    setType(type);
    active = false;
    bReset = false;
}


double Ramp::getVal(double time)
{
    if(!isActive() && bReset) 
    {
	start_time = time;
	active = true;
	return initial_data;
    }

    if(time < start_time + delta_time &&  delta_time!=0.0)
    {
	if(type == LINEAR)
	    return initial_data + ((final_data-initial_data)*(time-start_time)/delta_time);
    }
    else
    {
	bReset = false;
	active = false;	
	return final_data;
    }
}
