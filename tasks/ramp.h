/*
 * =====================================================================================
 *
 *       Filename:  ramp.h
 *
 *    Description:  RAMP UP or RAMP DOWN function
 *
 *        Version:  1.0
 *        Created:  02/04/10 10:55:43
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu (), ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#ifndef  RAMP_INC_INC
#define  RAMP_INC_INC

#define LINEAR 0
#define EXPONENTIAL 1  // not implemented

/*
 * =====================================================================================
 *        Class:  Ramp
 *  Description:  
 * =====================================================================================
 */
class Ramp
{
    public:
	/* ====================  LIFECYCLE     ======================================= */
	Ramp();                             /* constructor */
	Ramp(double initial_data, double final_data, double delta_time=0.0, int type=LINEAR);

	/* ====================  ACCESSORS     ======================================= */
	bool isActive() { return active; };
	double getVal(double curr_time);

	/* ====================  MUTATORS      ======================================= */
	void setDeltaTime(double val) { delta_time = val; };
	void setInitialData(double val) { initial_data = val; };
	void setFinalData(double val) { final_data = val; };
	void setType(int val) { type = val; };
	void reset(bool val = true) { bReset = val;  };

    protected:

    private:

	double initial_data;
        double final_data;
	double delta_time;
	double start_time;

	bool active;
	int type;
	bool bReset;

}; // -----  end of class Ramp

#endif   /* ----- #ifndef RAMP_INC_INC  ----- */
