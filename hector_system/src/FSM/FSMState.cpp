#include "../../include/FSM/FSMState.h"


FSMState::FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr):
            _data(data), _stateName(stateName), _stateNameStr(stateNameStr)
{
    _lowCmd = _data->_lowCmd;
    _lowState = _data->_lowState;
    


}





void FSMState::CheckJointSafety(){
    


    for (int leg = 0; leg < 2; leg++){

        // Hip Constraint
        if ((_data->_legController->data[leg].q(0) < _data->_biped->Abad_Leg_Constraint[0]) || 
          (_data->_legController->data[leg].q(0) > _data->_biped->Abad_Leg_Constraint[1])) {
            std::cout << "Abad R Angle Exceeded" << _data->_legController->data[0].q(0) << std::endl;
            abort();
        }

        // AbAd Constraint
        if ((_data->_legController->data[leg].q(1) < _data->_biped->Hip_Leg_Constraint[0]) ||
            (_data->_legController->data[leg].q(1) > _data->_biped->Hip_Leg_Constraint[1])) {
            std::cout << "Hip R Angle Exceeded" << std::endl;
            abort();
        }

        //Thigh Constraint
        if ((_data->_legController->data[leg].q(2) < _data->_biped->Thigh_Constraint[0]) || 
        (_data->_legController->data[leg].q(2) > _data->_biped->Thigh_Constraint[1])) {
            std::cout << "Thigh Angle Exceeded" << std::endl;
            abort();
        }

        //Calf Constraint
        if ((_data->_legController->data[leg].q(3) < _data->_biped->Calf_Constraint[0]) || 
        (_data->_legController->data[leg].q(3) > _data->_biped->Calf_Constraint[1])) {
            std::cout << "Calf Angle Exceeded" << std::endl;
            abort();
        }

        //Ankle Constraint
        if ((_data->_legController->data[leg].q(4) < _data->_biped->Ankle_Constraint[0]) || 
        (_data->_legController->data[leg].q(4) > _data->_biped->Ankle_Constraint[1])) {
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