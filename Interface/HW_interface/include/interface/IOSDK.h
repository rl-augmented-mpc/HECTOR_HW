#ifndef IOSDK_H
#define IOSDK_H

#include "IOInterface.h"
#include "../../../../hector_system/include/sdk/include/unitree_legged_sdk.h"
#include <fstream>

using namespace UNITREE_LEGGED_SDK;

class IOSDK : public IOInterface{
public:
IOSDK(LeggedType robot, int cmd_panel_id);
~IOSDK(){}
// void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
void sendRecv();


UDP _udp;
Safety _control;
LowCmd _lowCmd = {0};
LowState _lowState = {0};
};

#endif  // IOSDK_H