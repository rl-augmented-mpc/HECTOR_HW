
#include "../../include/interface/IOSDK.h"
#include "../../include/interface/WirelessHandle.h"
#include "../../include/interface/KeyBoard.h"
#include "../../../../hector_system/include/sdk/include/unitree_legged_sdk.h"
#include <stdio.h>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

IOSDK::IOSDK(LeggedType robot, int cmd_panel_id):
_control(robot),
_udp(LOWLEVEL)
{
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);
    if(cmd_panel_id == 1){
    cmdPanel = new WirelessHandle();
    }
    else if(cmd_panel_id == 2){
    cmdPanel = new KeyBoard();
    }

}

void IOSDK::sendRecv(){

    const LowlevelCmd *cmd = _data->_lowCmd;
    LowlevelState *state = _data->_lowState;

    _udp.Recv();
    _udp.GetRecv(_lowState);
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = 0X0A; 
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;
    }

    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;
    }
    
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.gyroscope[i]  = _lowState.imu.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    // for(int i = 0; i < 4; i++){
    //     state->FootForce[i] = _lowState.footForce[i];
    // }

    cmdPanel->receiveHandle(&_lowState);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    gaitNum = cmdPanel->getGaitNum();

    // _control.PowerProtect(_lowCmd, _lowState, 10);

    _udp.SetSend(_lowCmd);
    _udp.Send();
    // std::cout << "Execution time for sendRecv(): " << duration.count() << " microseconds" << std::endl;
}