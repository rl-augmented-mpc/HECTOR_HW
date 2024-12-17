#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Walking.h"
#include "FSMState_Passive.h"
#include "FSMState_PDStand.h"
#include "../common/enumClass.h"
// #include <SDL2/SDL.h>

// #include "../common/PoseData.h"

struct FSMStateList{
    FSMState *invalid;
    FSMState_Walking *walking;
    FSMState_Passive *passive;
    FSMState_PDStand *pdStand;
   
    void deletePtr(){
        delete invalid;
        delete walking;
        delete passive;
        delete pdStand;
    }  
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();



    private:
        FSMState* getNextState(FSMStateName stateName);
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;



};

#endif