#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "../common/ControlFSMData.h"
#include "../common/cppTypes.h"
#include "../common/enumClass.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"

class FSMState
{
    public:
        FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkTransition() {return FSMStateName::INVALID;}

        FSMStateName _stateName;
        std::string _stateNameStr;

        void CheckJointSafety();



    protected:
        
        ControlFSMData *_data;
        FSMStateName _nextStateName;

        LowlevelCmd *_lowCmd;
        LowlevelState *_lowState;


        



};

#endif // FSMSTATE_H