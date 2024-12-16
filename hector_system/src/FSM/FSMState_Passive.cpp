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
    // std::cout << "Current state is passive state" << std::endl;
    _data->_legController->updateData(_data->_lowState, offset); //Pass by reference?
    // _data->_stateEstimator->setContactPhase(contactphase);
    _data->_stateEstimator->run();

    joystick.pollEvents();
    state=joystick.getState();

    buttonA = state.buttons[0];
    buttonB = state.buttons[1];
    left_shoulder = state.buttons[4];

    if (left_shoulder) {
        abort();
    }


    for (int i = 0; i < 12; i++){
    angle << _lowState->motorState[i].q << "  ";
    }

    for (int leg = 0; leg < 2; leg++)
    {
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].q(i) << " ";
            corrected_angle << _data->_legController->data[leg].qd(i) << " ";
        }
    }

    angle << std::endl;
    corrected_angle<<std::endl;

    _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);

    com_pos << std::endl;
    rpy_input << std::endl;
}

void FSMState_Passive::exit()
{
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.setZero();
    }
    // _data->_interface->cmdPanel->setCmdNone();

    _data->_lowState->userValue.setZero();
    _data->_lowState->userCmd = UserCommand::NONE;
}

FSMStateName FSMState_Passive::checkTransition()
{
    if(_lowState->userCmd == UserCommand::WALK){
        std::cout << "transition from passive to Walking" << std::endl;
        return FSMStateName::WALKING;
    }
    else if (_lowState->userCmd == UserCommand::PDSTAND){
        return FSMStateName::PDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}