#include "../../include/FSM/FSMState_PDStand.h"
#include <chrono>

FSMState_PDStand::FSMState_PDStand(ControlFSMData *data)
                 :FSMState(data, FSMStateName::PDSTAND, "stand"),
                 pdStand() {}

void FSMState_PDStand::enter()
{
    // _data->_interface->zeroCmdPanel();
    _data->_lowState->userValue.setZero();
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();
}

void FSMState_PDStand::run()
{
    _data->_legController->updateData(_data->_lowState);
    CheckJointSafety();
    _data->_stateEstimator->run();





    //////////////////// PDStand ///////////////////
    pdStand.run(*_data); // run PD controller
    // PD results were directly stored in _data->_legController->command


    

    _data->_legController->updateCommand(_data->_lowCmd);  


}

void FSMState_PDStand::exit()
{      
    // _data->_interface->zeroCmdPanel();
    // _data->_interface->cmdPanel->setCmdNone();
    _data->_lowState->userValue.setZero();
    _data->_lowState->userCmd = UserCommand::NONE;
}

FSMStateName FSMState_PDStand::checkTransition()
{
    if(_lowState->userCmd == UserCommand::PASSIVE){
        _data->_legController->motiontime = 0;
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::WALK || _lowState->userCmd == UserCommand::STAND){
        _data->_legController->motiontime = 0;
        return FSMStateName::WALKING;
    }
    else {
        _data->_legController->motiontime++;
        return FSMStateName::PDSTAND;
    }
}