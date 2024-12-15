#include "../../include/interface/WirelessHandle.h"
#include <string.h>
#include <stdio.h>

WirelessHandle::WirelessHandle(){}

void WirelessHandle::receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){
    memcpy(&_keyData, lowState->wirelessRemote, 40);

    if(((int)_keyData.btn.components.L2 == 1) && 
       ((int)_keyData.btn.components.B  == 1)){
        userCmd = UserCommand::L2_B;
    }
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L2_A;
    }
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L2_X;
    }

    // else if(((int)_keyData.btn.components.L2 == 1) && 
    //         ((int)_keyData.btn.components.Y  == 1)){
    //     userCmd = UserCommand::L2_Y;
    // }

    // else if(((int)_keyData.btn.components.L1 == 1) && 
    //         ((int)_keyData.btn.components.X  == 1)){
    //     userCmd = UserCommand::L1_X;
    // }
    // else if(((int)_keyData.btn.components.L1 == 1) && 
    //         ((int)_keyData.btn.components.A  == 1)){
    //     userCmd = UserCommand::L1_A;
    // }
    // else if(((int)_keyData.btn.components.L1 == 1) && 
    //         ((int)_keyData.btn.components.Y  == 1)){
    //     userCmd = UserCommand::L1_Y;
    // }
    else if((int)_keyData.btn.components.start == 1){
        userCmd = UserCommand::START;
    }

    userValue.L2 = _keyData.L2;
    userValue.lx = _keyData.lx;
  
    userValue.ly = _keyData.ly;
  
    userValue.rx = _keyData.rx;
  
    userValue.ry =  _keyData.ry;

}
