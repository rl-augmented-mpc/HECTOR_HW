#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

class FSMState_Passive: public FSMState
{
    public:
        FSMState_Passive(ControlFSMData *data);
        void enter();
        void run();
        void exit();
        // double contactStateScheduled[2] = {1, 1};
        FSMStateName checkTransition();
};

#endif
