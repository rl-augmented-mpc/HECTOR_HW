#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "../common/ControlFSMData.h"
#include "../common/cppTypes.h"
#include "../common/enumClass.h"
#include "../interface/CmdPanel.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
// #include "../../include/common/PoseData.h"
#include "../interface/Joystick.h"
#include "../sdk/include/safety.h"
// #include "../../BalanceController/BalanceController.hpp"
// #include "../../ConvexMPC/ConvexMPCLocomotion.h"

class FSMState
{
    public:
        FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkTransition() {return FSMStateName::INVALID;}

        FSMStateName _stateName;
        std::string _stateNameStr;

        // Higher level Robot body controllers
        // BalanceController balanceController;
        // static float T265_pose[6];

        // static bool rs2_initialized;

        // PoseData pd;
        // static rs2::pipeline pipe;
        // rs2::config cfg;

        double *Angle_Caliberation();

        double* offset;


    protected:

        // Holds all off the relevant control data
        // UNITREE_LEGGED_SDK::Safety control;

        // ConvexMPCLocomotion Cmpc;
        
        ControlFSMData *_data;
        FSMStateName _nextStateName;

        LowlevelCmd *_lowCmd;
        LowlevelState *_lowState;
        UserValue _userValue;

        int motionTime = 0;
        double dt;
        // double* offset;
        
        //Todo, add static, want to add to fsm.cpp so that run everytime
        Joystick joystick;
        JoystickState state;

        double xAxis;
        double yAxis;
        double zAxis;

        double vx_command;
        double vy_command;

        bool buttonA;
        bool buttonB;
        bool buttonX;
        bool left_shoulder;
        bool flagWalk=0;
        bool flagZeroVel;
        bool flagHi;
        double motionTimeHi;
        bool flagGaitTimer_Stand;
        bool flagGaitTimer_Walk;




        Vec2<double> contactphase = {0.5,0.5}; // default contact phase



        // Desired States
        // double roll = 0;
        // double pitch = 0;
        // double yaw_rate = 0;


        //Data Recording
        // Vec6<double> pFeetDes;

        // double fOpt[12];

        // bool Calibration = true;

        //Angle Constraint
        double Abad_Leg1_Constraint[2] = {-30 * (3.1415/180), 30 * (3.1415/180)};
        double Abad_Leg2_Constraint[2] = {-30 * (3.1415/180), 30 * (3.1415/180)};
        double Hip_Leg1_Constraint[2] = {-25* (3.1415/180.0), 55 * (3.1415/180)};
        double Hip_Leg2_Constraint[2] = {-25 * (3.1415/180), 25 * (3.1415/180)};
        double Thigh_Constraint[2] = {0 * (3.1415/180), 80 * (3.1415/180)};
        double Calf_Constraint[2] = {-150 * (3.1415/180), -30 * (3.1415/180)};
        double Ankle_Constraint[2] = {-10 * (3.1415/180), 95 * (3.1415/180)};

        // double init_yaw = 0;
        // double target_time = 10000;

        //Actual and Desired Foot Position Vector Calculation
        // Vec3<double> pFeetVecCOM;

        std::ofstream myfile;
        std::ofstream QP;
        std::ofstream com_pos;
        std::ofstream b_des;
        std::ofstream angle;
        std::ofstream torque;
        std::ofstream footposition;
        std::ofstream rpy_input;
        std::ofstream force;
        std::ofstream omega;
        std::ofstream acceleration;
        std::ofstream tau_est;
        std::ofstream temperature;
        std::ofstream corrected_angle;
        std::ofstream T265_pos;
        std::ofstream T265_qua;
        std::ofstream fullStateTraj;
        // Vec3<double> fOptR;
        // Vec3<double> fOptO;
        // Vec3<double> mOptR;
        // Vec3<double> mOptO;
};

#endif // FSMSTATE_H