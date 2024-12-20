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
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run();
    _data->_legController->zeroCommand();
    Cmpc.firstRun = true;

    _data->_lowState->userValue.setZero();
}

void FSMState_Walking::run()
{
    _data->_legController->updateData(_data->_lowState);
    CheckJointSafety();
    _data->_stateEstimator->run();






    //////////////////// MPC ///////////////////
    // pull velocity from keyboard (see src/interface/KeyBoard.cpp for key mappings)
    // message from Ziwon: Let's keep this structure of using UserValue and UserCmd
    UserValue _userValue;
    _userValue = _data->_lowState->userValue;
    v_des_body[0] = (double)_userValue.lx;
    v_des_body[1] = (double)_userValue.ly;
    turn_rate = (double)_userValue.rx;

    int gaitNum = 1;
    if (_data->_lowState->userCmd == UserCommand::WALK){ 
        gaitNum = 2; // walking
    }
    if(_data->_lowState->userCmd == UserCommand::STAND){
        gaitNum = 1; // stand
    }

    Cmpc.setGaitNum(gaitNum);
    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    Cmpc.run(*_data);







    Logging();
    //Push the Command to Leg Controller
    _data->_legController->updateCommand(_data->_lowCmd);





}

void FSMState_Walking::exit()
{
    counter = 0;
    _data->_lowState->userValue.setZero();
    _data->_lowState->userCmd = UserCommand::NONE;
}

FSMStateName FSMState_Walking::checkTransition()
{

    if(_lowState->userCmd == UserCommand::PASSIVE){
        _data->_legController->motiontime = 0;
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::PDSTAND){
        _data->_legController->motiontime = 0;
        return FSMStateName::PDSTAND;
    }
    else{
        _data->_legController->motiontime++;
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



