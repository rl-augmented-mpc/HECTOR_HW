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


        command

        // Extract data from data._legController->data
        q_curr = data._legController->data[i].q;
        
        // Calculate error
        for (int j = 0; j < 5; j++) {
            error[j] = q_target[j] - q_curr[j];

            if (firstRun) { //init prevError values
                prevError[i][j] = error[j];
                firstRun = false;
                std::cout << "init prevError" << firstRun << std::endl;
            }
        }

        // Calculate control signal
        for (int j = 0; j < 5; j++) {
            controlSignal[j] = kp[j] * error[j] + kd[j] * (error[j] - prevError[i][j]) / dt;
        }

        // Assign output control values to data._legController->commands
        for (int j = 0; j < 5; j++) {
            data._legController->commands[i].tau[j] = controlSignal[j];
            // data._legController->commands[i]->qDes[j] = q_target[j];
            // data._legController->commands[i]->qdDes[j] = controlSignal[j];
        }

        // Update previous error
        prevError[i] = error;

        // debug statements
        if (i == 0) {
            // std::cout << "qTargetLeft: \n" << q_target << std::endl;
            // std::cout << "qCurrLeft: \n" << q_curr << std::endl;
            std::cout << "errorLeft: \n" << error << std::endl;
            std::cout << "controlSignalLeft: \n" << controlSignal << std::endl;
        }
        else {
            // std::cout << "qTargetRight: \n" << q_target << std::endl;
            // std::cout << "qCurrRight: \n" << q_curr << std::endl;
            std::cout << "errorRight: \n" << error << std::endl;
            std::cout << "controlSignalRight: \n" << controlSignal << std::endl;
            std::cout << std::endl;
        }   
    }
}
