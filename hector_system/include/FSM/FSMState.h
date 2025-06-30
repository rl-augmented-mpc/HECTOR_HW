#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>


#include <iostream>
#include <fstream>
#include <iomanip>  // For std::put_time
#include <ctime>    // For std::localtime
#include <chrono>   // For std::chrono::system_clock


#include "../common/ControlFSMData.h"
#include "../common/cppTypes.h"
#include "../common/enumClass.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"

class FSMState
{
    public:
        FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        virtual void reset() = 0;
        virtual FSMStateName checkTransition() {return FSMStateName::INVALID;}

        FSMStateName _stateName;
        std::string _stateNameStr;

        void CheckJointSafety();
        void Logging(ConvexMPCLocomotion &mpc);

        // Desired States (only used in walking, but make it shared variable)
        Vec3<double> v_des_body{0, 0, 0};
        double turn_rate = 0;
        double roll = 0;
        double pitch = 0;
        double reference_height=0.53;
        int gaitNum = 1; // 1: stand, 2: walk


        //For Logging=======================================================
        std::ofstream sensor_data;
        int idx_imu = 0;
        int idx_imu_bias = idx_imu+6;
        int idx_joint_pos = idx_imu_bias+6;
        int idx_joint_vel = idx_joint_pos+10;
        int idx_joint_tau = idx_joint_vel+10;
        int idx_foot_pos = idx_joint_tau + 10; 
        int idx_foot_vel = idx_foot_pos+6;
        int idx_sensordata_end = idx_foot_vel+6;
        const static int sensor_data_size = 54; //sum of all aboves
        double sensor_data_array [sensor_data_size] = {0};

        std::ofstream estimated_data;
        int idx_com_rpy   = 0;
        int idx_com_omega = idx_com_rpy+3;
        int idx_com_pos   = idx_com_omega+3;
        int idx_com_vel   = idx_com_pos+3;
        int idx_com_acc   = idx_com_vel+3;
        int idx_foot_left_contact = idx_com_acc+3;
        int idx_foot_right_contact = idx_foot_left_contact+1;
        int idx_estimated_data_end = idx_foot_right_contact+1;
        const static int estimated_data_size = 17; //sum of all aboves
        double estimated_data_array [estimated_data_size] = {0};

        std::ofstream controller_data;
        int idx_GRF_left = 0;
        int idx_GRF_right = idx_GRF_left+3;
        int idx_GRM_left = idx_GRF_right+3;
        int idx_GRM_right = idx_GRM_left+3;

        int idx_joint_cmd_tau = idx_GRM_right+3;
        int idx_joint_cmd_pos = idx_joint_cmd_tau+10;
        int idx_joint_cmd_vel = idx_joint_cmd_pos+10; 

        int idx_ref_com_rpy = idx_joint_cmd_vel+10;
        int idx_ref_com_omega = idx_ref_com_rpy +3;
        int idx_ref_com_pos   = idx_ref_com_omega+3;
        int idx_ref_com_vel   = idx_ref_com_pos+3;
        
        int idx_controller_data_end = idx_ref_com_vel+3;
        const static int controller_data_size = 54; //sum of all aboves
        double controller_data_array [controller_data_size] = {0};

    protected:
        
        ControlFSMData *_data;
        FSMStateName _nextStateName;

        LowlevelCmd *_lowCmd;
        LowlevelState *_lowState;


        



};

#endif // FSMSTATE_H