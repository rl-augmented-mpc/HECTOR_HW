#include "PDController_Stand.h"

// TODO KD and KP for indiv joints
// do some jacobian shit
// clean up code

PDController_Stand::PDController_Stand() {
    firstRun = true;
    kp << 100, 100, 100, 100, 100; // 100,180,300
    kd << 1, 1, 1, 1, 1; // 3,8,15
    q_target << 0, 0, 0.942477, -1.88495, 0.942477; // starting pos
    // q_target << 0, 0, 55.0/180.0*3.14, -100.0/180.0*3.14, 45.0/180.0*3.14;
    //0 ,0 ,55, -100, 45 yaw,roll,pitch,calf,ankle
}

void PDController_Stand::run(ControlFSMData& data) {
    double dt = 0.001; // 0.001

    // loop for both legs
    for (int i = 0; i < 2; i++) {

        for (int k = 0; k < 5; k ++) {
            data._legController->commands[i].qDes[k] = q_target[k];
            data._legController->commands[i].qdDes[k] = 0;
            data._legController->commands[i].tau[k] = 0;
        }



        data._legController->commands[i].feedforwardForce << 0,0,0,0,0,0;


        data._legController->commands[i].control_mode =  int(ControlMode::PDSTAND);






    }
}
