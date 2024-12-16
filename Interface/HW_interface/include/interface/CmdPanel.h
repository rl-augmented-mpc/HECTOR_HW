#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "../../../../hector_system/include/messages/unitree_joystick.h"
#include "../../../../hector_system/include/common/enumClass.h"
#include "../sdk/include/unitree_legged_sdk.h"
#include "../../../../hector_system/include/messages/LowlevelState.h"
#include <pthread.h>


class CmdPanel{
public:
    CmdPanel(){}
    ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    UserCommand getGaitNum(){return gaitNum;}
    void setPassive(){userCmd = UserCommand::PASSIVE;}
    void setZero(){userValue.setZero();}
    void setCmdNone(){userCmd = UserCommand::NONE;}
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
    virtual void setHighlevelMsg(UNITREE_LEGGED_SDK::HighState *highState){};
protected:
    // virtual void *run(void *arg);
    UserCommand userCmd;
    UserValue userValue;
    UserCommand gaitNum = UserCommand::NONE;
};

#endif  // CMDPANEL_H
