#ifndef WALKING_H
#define WALKING_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"

class FSMState_Walking: public FSMState
{
    public:
        FSMState_Walking(ControlFSMData *data, int cmd_mode);
        ~FSMState_Walking(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
        void setCommand();

    private:
        ConvexMPCLocomotion Cmpc;
        int _cmd_mode = 0; // 0: keyboard, 1: joystick
        int counter;

        
        // Desired States
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
        Eigen::VectorXd trajectory;
        Eigen::VectorXd contactState;
        Eigen::VectorXd getTrajectory();



};

#endif