#include "../../include/FSM/FSM.h"
#include <iostream>

FSM::FSM(ControlFSMData *data)
    :_data(data)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.walking = new FSMState_Walking(_data, 0); // 0: keyboard, 1: joystick for cmd_mode
    _stateList.pdStand = new FSMState_PDStand(_data); //PDStanding!
    // add other FSM states later


    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize()
{
    count = 0;
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;

}

void FSM::run()
{
    // _data->sendRecv();

    if(!checkSafty())
    {
        // _data->_interface->setPassive(); // set to L2_B
        _data->_lowState->userCmd = UserCommand::PASSIVE;
    }
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

bool FSM::checkSafty()
{
    if(_data->_stateEstimator->getResult().rBody(2,2) < 0.5) // TODO: Does this need to be changed?
    {
        return false;
    }
    else
    {
        return true;
    }
}
