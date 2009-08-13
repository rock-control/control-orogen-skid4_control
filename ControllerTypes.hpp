#ifndef CONTROLLER_TYPES_HPP
#define CONTROLLER_TYPES_HPP

namespace controller
{
    struct TwoPhaseMotorState
    {
        double touchdown_time;
        int phase;
        int last_phase;
    }; 

    struct TwoPhaseState
    {
        TwoPhaseMotorState motors[4];
    };
}

#endif

