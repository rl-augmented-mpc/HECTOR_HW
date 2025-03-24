#include "../../include/FSM/FSM.h"
#include <iostream>

// FSM::FSM(ControlFSMData *data)
//     :_data(data)
// {
//     _stateList.invalid = nullptr;
//     _stateList.passive = new FSMState_Passive(_data);
//     _stateList.walking = new FSMState_Walking(_data, 0); // 0: keyboard, 1: joystick for cmd_mode
//     _stateList.pdStand = new FSMState_PDStand(_data); //PDStanding!
//     // add other FSM states later


//     initialize();
// }

FSM::FSM(
    ControlFSMData *data, double _dt, int _iterations_between_mpc, 
    int _horizon_length, int _mpc_decimation, std::string fsm_name)
    :_data(data)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new FSMState_Passive(_data); // Passive
    _stateList.walking = new FSMState_Walking(_data, _dt, _iterations_between_mpc, _horizon_length, _mpc_decimation); // Walking
    _stateList.pdStand = new FSMState_PDStand(_data); //PDStanding
    // add other FSM states later
    initialize(fsm_name);
}

FSM::~FSM(){
    _stateList.deletePtr();
}

// void FSM::initialize()
// {
//     count = 0;
//     _currentState = _stateList.passive;
//     _currentState -> enter();
//     _nextState = _currentState;
//     _mode = FSMMode::NORMAL;

// }

void FSM::initialize(std::string _fsm_name)
{
    count = 0;
    if (_fsm_name == "walking")
    {
        _currentState = _stateList.walking;
    }
    else if (_fsm_name == "pdStand")
    {
        _currentState = _stateList.pdStand;
    }
    else
    {
        _currentState = _stateList.passive;
    }
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;

}

void FSM::run()
{

    if(_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkTransition();
        if(_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    else if(_mode == FSMMode::CHANGE)
    {
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();       
    }
    count++;
}

FSMState* FSM::getNextState(FSMStateName stateName)
{
    switch(stateName)
    {
        case FSMStateName::INVALID:
            return _stateList.invalid;
        break;
        case FSMStateName::PASSIVE:
            return _stateList.passive;
        break;
        case FSMStateName::WALKING:
            return _stateList.walking;
        break;
        case FSMStateName::PDSTAND: // new state for PDStand
            return _stateList.pdStand;
        break;
        default:
            return _stateList.invalid;
        break;
    }
}


// ** simulation specific methods
void FSM::setStateCommands(Vec2<double> roll_pitch, Vec3<double> twist, double ref_height){
    _currentState->roll = roll_pitch[0];
    _currentState->pitch = roll_pitch[1];
    _currentState->v_des_body[0] = twist[0]*std::cos(_data->_biped->slope_pitch);
    _currentState->v_des_body[1] = twist[1];
    _currentState->v_des_body[2] = twist[0]*std::sin(_data->_biped->slope_pitch); 
    _currentState->turn_rate = twist[2];
    _currentState->reference_height = ref_height;
}

void FSM::setGaitNum(int gaitNum){
    _currentState->gaitNum = gaitNum;
}

void FSM::reset(){
    _currentState->reset();
}
