#include "../../include/FSM/FSMState_Walking.h"
#include <chrono>

FSMState_Walking::FSMState_Walking(ControlFSMData *data, int cmd_mode)
                 :FSMState(data, FSMStateName::WALKING, "walking"),
                 _cmd_mode(cmd_mode),
                 Cmpc(0.001, 50){}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

void FSMState_Walking::enter()
{   
    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    // _data->_interface->zeroCmdPanel(); // reset UserValue to all zeros
    _data->_lowState->userValue.setZero();

    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run();
    _data->_legController->zeroCommand();
    Cmpc.firstRun = true;
    // motionTime = 5000;
}

void FSMState_Walking::run()
{
    auto start = std::chrono::steady_clock::now();
    motionTime++;
    std::cout << "Current state is MPC " << std::endl;
    _data->_legController->updateData(_data->_lowState, offset);
    _data->_stateEstimator->run();
    _userValue = _data->_lowState->userValue;

    std::cout << "motiontime is " << motionTime << std::endl;

    // Joystick operation mode //
    int gaitNum = 1; // standing default
    int gaitTime = 250; //0.25s


    // pull velocity from keyboard (see src/interface/KeyBoard.cpp for key mappings)
    v_des_body[0] = (double)_userValue.lx;
    v_des_body[1] = (double)_userValue.ly;
    turn_rate = (double)_userValue.rx;

    // set gait number
    if (_data->_lowState->userCmd == UserCommand::WALK){ 
        flagGaitTimer_Walk = 1;
    }
    if(flagGaitTimer_Walk == 1 && motionTime%(gaitTime) == gaitTime/2){
        flagWalk = 1;
        flagGaitTimer_Walk = 0;
    }

    if(_data->_lowState->userCmd == UserCommand::STAND){
        flagGaitTimer_Stand = 1;
    }
    if(flagGaitTimer_Stand == 1 && motionTime%gaitTime == 0){
        flagWalk = 0;
        flagGaitTimer_Stand = 0;
    }
    
    
    // if (flagWalk==0){
    //     std::cout << "standing mode" << std::endl; 
    // }
    // else if (flagWalk == 1){
    //     std::cout << "walking mode" << std::endl; 
    // }



    if (left_shoulder) {
        abort();
    }

    if(flagWalk == 0){
        gaitNum = 1; // standing
    }

    // rebalance after disturbance:
    // if (motionTime > 5000){
    //     for (int i=0; i<3; i++){
    //         p_act[i] = _data->_stateEstimator->getResult().position(i);
    //         footCenter[i] = (_data->_legController->data[0].p[i] + _data->_legController->data[1].p[i])/2;
    //     }

    //     if (abs(footCenter[0]) > balanceToleranceX){
    //         if (flagRebalance == 0) {
    //             motionTimeRebalance = motionTime;
    //         }
    //         flagRebalance = 1;
    //     }

    //     if (motionTime < motionTimeRebalance + 1350){
    //         gaitNum = 3;
    //     }
    //     else if (motionTime >= motionTimeRebalance + 1350 && motionTime <= motionTimeRebalance + 1800){
    //         gaitNum = 7;
    //     }
    //     else if (motionTime > motionTimeRebalance + 1800){
    //         flagRebalance = 0;
    //     }
    // }

    if(flagWalk == 1){
        gaitNum = 2; // walking
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

    //////////////////// MPC ///////////////////
    if(motionTime >= 0){
        Cmpc.setGaitNum(gaitNum);
        _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
        Cmpc.run(*_data);

        std::cout << "vx, vy, wz: " << v_des_body[0] << " " << v_des_body[1] << " " << turn_rate << std::endl;

        //Push the Command to Leg Controller
        _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);

    }


    // //Data Recording
    contactState = Cmpc.contact_state;

    fullStateTraj << getTrajectory().transpose() << std::endl;


    for (int i = 0; i <3; i++){
        com_pos << _data->_stateEstimator->getResult().position(i) << "  ";
        rpy_input << _data->_stateEstimator->getResult().rpy(i) << " ";
    }

    for (int i = 0; i < 3; i++){
        com_pos << _data->_stateEstimator->getResult().vWorld(i) << "  ";
        rpy_input << _data->_stateEstimator->getResult().omegaWorld(i) << " ";
    }

 

    for (int leg = 0; leg < 2; leg++){
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].q(i) << " ";
        }
    }
    for (int leg = 0; leg < 2; leg++){
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].qd(i) << " ";
        }
    }

    for (int i = 0; i < 4; i++){
        T265_qua << _data->_stateEstimator->getResult().orientation(i) << " ";
    }

    // for (int i = 0; i < 12; i++){
    //     tau_est << _data->_lowState->motorState->tauEst[i] << " ";
    // }

    angle << std::endl;
    torque << std::endl;
    com_pos << std::endl;
    footposition << std::endl;
    QP << std::endl;
    myfile << std::endl;
    force << std::endl;
    rpy_input << std::endl;
    b_des << std::endl;
    tau_est << std::endl;
    corrected_angle<<std::endl;
    T265_pos << std::endl;
    T265_qua << std::endl;
    auto end =std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Execution time for FSM_walk(): " << duration.count() << " microseconds" << std::endl;


}

void FSMState_Walking::exit()
{
    // _data->_interface->zeroCmdPanel(); // set uservalue to 0
    // _data->_interface->cmdPanel->setCmdNone(); // set cmd to none
    _data->_lowState->userValue.setZero();
    _data->_lowState->userCmd = UserCommand::NONE;
}

FSMStateName FSMState_Walking::checkTransition()
{

    if(_lowState->userCmd == UserCommand::PASSIVE){
        std::cout << "transition from walking to passive" << std::endl;
        // abort();
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::PDSTAND){
        return FSMStateName::PDSTAND;
    }
    else{
        return FSMStateName::WALKING;
    }
}


Eigen::VectorXd FSMState_Walking::getTrajectory(){
  
  // Roll out MPC trajectory desired structure of trajectory is as follows
  // mpcTraj.row() = {x,  y,  z,  r,  p,  y,  q[left],  q[right], 
  //                  vx, vy, vz, wx, wy, wz, wq[left], wq[right], 
  //                  contact[left], contact[right]}
  int totalSize = 34; 
  trajectory.resize(totalSize);
  trajectory.setZero();

  // Position and Velocity
  for (int i = 0; i < 3; i++) {
    trajectory(i) = _data->_stateEstimator->getResult().position(i);
    trajectory(i + 3) = _data->_stateEstimator->getResult().rpy(i);
  }

  // Orientation and Angular Velocity
  for (int i = 0; i < 3; i++) {
    trajectory(i + 16) = _data->_stateEstimator->getResult().vWorld(i);; 
    trajectory(i + 19) = _data->_stateEstimator->getResult().omegaWorld(i);
  }

  // Joint Angles and Velocities
  for (int leg = 0; leg < 2; leg++) {
    for (int i = 0; i < 5; i++) {
      trajectory(6 + leg*5 + i) = _data->_legController->data[leg].q(i);
      trajectory(22 + leg*5 + i) = _data->_legController->data[leg].qd(i);
    }
  }

  // Contact States
  trajectory(totalSize - 2) = contactState(0); 
  trajectory(totalSize - 1) = contactState(1);
  return trajectory;
}
