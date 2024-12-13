#include "../../include/FSM/FSM.h"
#include <iostream>

FSM::FSM(ControlFSMData *data)
    :_data(data)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.walking = new FSMState_Walking(_data, 0); // 0: keyboard, 1: joystick for cmd_mode
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
        _data->_interface->setPassive(); // set to L2_B
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

// double *FSM::Angle_Caliberation(){
//     std::ifstream angle_file;
//     std::string angle_name;
//     int i = 0;

//     angle_file.open("../../Calibration/offset.txt");

//     getline(angle_file, angle_name);

//     std::stringstream ss(angle_name);
//     double angle1, angle2, angle3, angle4, angle5, 
//             angle6, angle7, angle8, angle9, angle10;
//     ss >> angle1 >> angle2 >> angle3 >> angle4 >> angle5 >> angle6 >> angle7 >> angle8 >> angle9 >> angle10;

//     static double offset_angle[10] = {angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9, angle10};

//     return offset_angle;
// }

// double *FSM::rpy_Caliberation(){
//     std::ifstream rpy_file;
//     std::string rpy_name;
//     int i = 0;
//     static double rotm[10];

//     rpy_file.open("../../Calibration/rpy_offset.txt");

//     while (getline(rpy_file, rpy_name))
//     {
//         std::stringstream sss(rpy_name);
//         sss >> rotm[i] >> rotm[i+1] >> rotm[i+2];
//         i = i+3;
//     }

//     return rotm;
// }