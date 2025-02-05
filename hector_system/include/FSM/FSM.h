#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Walking.h"
#include "FSMState_Passive.h"
#include "FSMState_PDStand.h"
#include "../common/enumClass.h"

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
        // FSM(ControlFSMData *data);
        FSM(ControlFSMData *data, double _dt, int _iterations_between_mpc, 
            int _horizon_length, int _mpc_decimation, std::string fsm_name);
        ~FSM();
        // void initialize();
        void initialize(std::string _fsm_name);
        void run();
        void reset();

        // Helper functions for simulation to set command.
        void setStateCommands(Vec2<double> roll_pitch, Vec3<double> twist, double ref_height); 
        void setGaitNum(int gaitNum);


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