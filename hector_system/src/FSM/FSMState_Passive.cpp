#include "../../include/FSM/FSMState_Passive.h"

FSMState_Passive::FSMState_Passive(ControlFSMData *data):
                  FSMState(data, FSMStateName::PASSIVE, "passive"){}

void FSMState_Passive::enter()
{
    _data->_legController->zeroCommand();
    _data->_stateEstimator->run(); 

    for(int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 5; j++){
            _data->_legController->commands[i].kdJoint(j) = 5;
        }
    }

}

void FSMState_Passive::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run();

    for (int leg = 0; leg < 2; leg++)
    {
        _data->_legController->commands[leg].which_control = 0;
    }

    _data->_legController->updateCommand(_data->_lowCmd);


    //needed for joint angle calibration
    std::ofstream angle;
    angle.open("angle.txt");
    for (int i = 0; i < 12; i++){
        angle << _lowState->motorState[i].q << "  ";
    }
    angle << std::endl;

}

void FSMState_Passive::exit()
{
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.setZero();
    }

    _data->_lowState->userValue.setZero();
    _data->_lowState->userCmd = UserCommand::NONE;
}

FSMStateName FSMState_Passive::checkTransition()
{
    if(_lowState->userCmd == UserCommand::WALK || _lowState->userCmd == UserCommand::STAND){
        _data->_legController->motiontime = 0;
        return FSMStateName::WALKING;
    }
    else if (_lowState->userCmd == UserCommand::PDSTAND){
        _data->_legController->motiontime = 0;
        return FSMStateName::PDSTAND;
    }
    else{
        _data->_legController->motiontime++;
        return FSMStateName::PASSIVE;
    }
}