#include "../../include/FSM/FSMState_PDStand.h"

FSMState_PDStand::FSMState_PDStand(ControlFSMData *data)
                 :FSMState(data, FSMStateName::PDSTAND, "stand"),
                 pdStand() {}

void FSMState_PDStand::enter()
{
    _data->_interface->zeroCmdPanel();
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();
}

void FSMState_PDStand::run()
{
    auto start = std::chrono::steady_clock::now();
    motionTime++;
    _data->_legController->updateData(_data->_lowState, offset); //getting joint state
    _data->_stateEstimator->run(); 
    _userValue = _data->_lowState->userValue;
    std::cout << "motiontime is " << motionTime << std::endl;







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

        //Angle Constraints
    if (motionTime > 50){
        // Hip Constraint
        if ((_data->_legController->data[0].q(0) < Abad_Leg1_Constraint[0]) || 
          (_data->_legController->data[0].q(0) > Abad_Leg1_Constraint[1])) {
            std::cout << "Abad R Angle Exceeded" << _data->_legController->data[0].q(0) << std::endl;
            abort();
          }
        if ((_data->_legController->data[1].q(0) < Abad_Leg2_Constraint[0]) || 
            (_data->_legController->data[1].q(0) > Abad_Leg2_Constraint[1])) {
            std::cout << "Abad L Angle Exceeded" << _data->_legController->data[1].q(0) << std::endl;
            abort();
            }

        // AbAd Constraint
        if ((_data->_legController->data[0].q(1) < Hip_Leg1_Constraint[0]) ||
            (_data->_legController->data[0].q(1) > Hip_Leg1_Constraint[1])) {
            std::cout << "Hip R Angle Exceeded" << std::endl;
            abort();
            }
        if ((_data->_legController->data[1].q(1) < Hip_Leg2_Constraint[0]) ||
            (_data->_legController->data[1].q(1) > Hip_Leg2_Constraint[1])) {
            std::cout << "Hip L Angle Exceeded" << std::endl;
            abort();
            }

        //Thigh Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(2) < Thigh_Constraint[0]) || 
            (_data->_legController->data[leg].q(2) > Thigh_Constraint[1])) {
                std::cout << "Thigh Angle Exceeded" << std::endl;
                abort();
            }
        }

        //Calf Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(3) < Calf_Constraint[0]) || 
            (_data->_legController->data[leg].q(3) > Calf_Constraint[1])) {
                std::cout << "Calf Angle Exceeded" << std::endl;
                abort();
            }
        }

        //Ankle Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(4) < Ankle_Constraint[0]) || 
            (_data->_legController->data[leg].q(4) > Ankle_Constraint[1])) {
                std::cout << "Ankle Angle Exceeded" << std::endl;
                abort();
            }
        }

        //Pitch Constraint
        if ((_data->_stateEstimator->getResult().rpy(1)) < -0.3){
            std::cout << "Pitch Angle Exceeded" << std::endl;
            abort();
        }
    }


    //////////////////// PDStand ///////////////////
    if(motionTime >= 0){

        // set desired state, all set to 0 by default in FSMState_PDStand.h
        _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
        
        pdStand.run(*_data); // run PD controller

        // PD results were directly stored in _data->_legController->command
        // They are converted and transited to _data->lowCmd
        _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);  


    }

}

void FSMState_PDStand::exit()
{      
    _data->_interface->zeroCmdPanel();
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_PDStand::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::WALKING;
    }
    else {
        return FSMStateName::PDSTAND;
    }
}