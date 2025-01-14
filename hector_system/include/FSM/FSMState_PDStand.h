#ifndef STANDING_H
#define STANDING_H

#include "FSMState.h"
#include "../../PDController/PDController_Stand.h"

class FSMState_PDStand: public FSMState
{
    public:
        FSMState_PDStand(ControlFSMData *data);
        ~FSMState_PDStand(){}
        void enter();
        void run();
        void exit();
        void reset(){;}; // do nothing
        FSMStateName checkTransition();
    
    private:
        PDController_Stand pdStand;
};

#endif