#include "../../include/FSM/FSMState.h"


FSMState::FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr):
            _data(data), _stateName(stateName), _stateNameStr(stateNameStr)
{
    _lowCmd = _data->_lowCmd;
    _lowState = _data->_lowState;
    



    //For Logging=======================================================

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");

    std::string currentTime = ss.str();
    std::string realorsim;
    if (_data->_biped->_real_flag == 1)
    {
        realorsim = "Real_";
    }
    else
    {
        realorsim = "Sim_";
    }
    // std::string targetDir = "../../results/" + realorsim + currentTime;
    std::string targetDir = "../results/" + realorsim + currentTime;

    // Create the target directory using mkdir command to ensure it exists
    std::string mkdirCommand = "mkdir -p " + targetDir;
    int ret = system(mkdirCommand.c_str());
    if (ret != 0)
    {
        std::cerr << "Failed to make dir: " << targetDir << std::endl;
    }

    std::string chmodCommand = "chmod -R a+rwx " + targetDir;
    ret = system(chmodCommand.c_str());
    if (ret != 0)
    {
        std::cerr << "Failed to chmod dir: " << targetDir << std::endl;
    }

    sensor_data.open(targetDir + "/sensor_data.txt");
    estimated_data.open(targetDir + "/estimated_data.txt");
    controller_data.open(targetDir + "/controller_data.txt");

    //For Logging=======================================================



}

void FSMState::Logging()
{

    // Data Recording

    // LIDAR added=======================================================
    for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    {
        sensor_data_array[idx_imu + temp_idx] = _data->_stateEstimator->getResult().omegaWorld(temp_idx);
        sensor_data_array[idx_imu + 3 + temp_idx] = _data->_stateEstimator->getResult().aWorld(temp_idx);
        // sensor_data_array[idx_imu_bias + temp_idx] = _data->_stateEstimator->getResult().gyro_bias(temp_idx);
        // sensor_data_array[idx_imu_bias + 3 + temp_idx] = _data->_stateEstimator->getResult().acc_bias(temp_idx);
    }
    for (int temp_idx = 0; temp_idx < 5; temp_idx++)
    {
        sensor_data_array[idx_joint_pos + temp_idx] = _data->_legController->data[0].q(temp_idx);       // left, already calibrated
        sensor_data_array[idx_joint_pos + 5 + temp_idx] = _data->_legController->data[1].q(temp_idx);   // right
        sensor_data_array[idx_joint_vel + temp_idx] = _data->_legController->data[0].qd(temp_idx);      // left
        sensor_data_array[idx_joint_vel + 5 + temp_idx] = _data->_legController->data[1].qd(temp_idx);  // right
        sensor_data_array[idx_joint_tau + temp_idx] = _data->_legController->data[0].tau(temp_idx);     // left
        sensor_data_array[idx_joint_tau + 5 + temp_idx] = _data->_legController->data[1].tau(temp_idx); // right
    }

    // Vec3<double> temp_rpy_left = BasicFunctions::Rotation_to_EulerZYX(_data->_stateEstimator->getResult().RFootWorld[0]);
    // Vec3<double> temp_rpy_right = BasicFunctions::Rotation_to_EulerZYX(_data->_stateEstimator->getResult().RFootWorld[1]);
    for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    {
        estimated_data_array[idx_com_rpy + temp_idx] = _data->_stateEstimator->getResult().rpy(temp_idx);
        estimated_data_array[idx_com_omega + temp_idx] = _data->_stateEstimator->getResult().omegaWorld(temp_idx);
        estimated_data_array[idx_com_pos + temp_idx] = _data->_stateEstimator->getResult().position(temp_idx); // or T265_pose
        estimated_data_array[idx_com_vel + temp_idx] = _data->_stateEstimator->getResult().vWorld(temp_idx);
        estimated_data_array[idx_com_acc + temp_idx] = _data->_stateEstimator->getResult().aWorld(temp_idx);

        // estimated_data_array[idx_foot_left_rpy + temp_idx] = temp_rpy_left(temp_idx);
        // estimated_data_array[idx_foot_left_omega + temp_idx] = _data->_stateEstimator->getResult().omegaFootWorld[0](temp_idx);
        // estimated_data_array[idx_foot_left_pos + temp_idx] = _data->_stateEstimator->getResult().pFootWorld[0](temp_idx);
        // estimated_data_array[idx_foot_left_vel + temp_idx] = _data->_stateEstimator->getResult().vFootWorld[0](temp_idx);

        // estimated_data_array[idx_foot_right_rpy + temp_idx] = temp_rpy_right(temp_idx);
        // estimated_data_array[idx_foot_right_omega + temp_idx] = _data->_stateEstimator->getResult().omegaFootWorld[1](temp_idx);
        // estimated_data_array[idx_foot_right_pos + temp_idx] = _data->_stateEstimator->getResult().pFootWorld[1](temp_idx);
        // estimated_data_array[idx_foot_right_vel + temp_idx] = _data->_stateEstimator->getResult().vFootWorld[1](temp_idx);

        // estimated_data_array[idx_foot_left_contact] -> contact estimator not implemented yet.
    }

    // for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    // {
    //     controller_data_array[idx_GRF_left + temp_idx] = GMmpc.GetGMMPCSolution(temp_idx); // world frame. should be in foot frame
    //     controller_data_array[idx_GRF_right + temp_idx] = GMmpc.GetGMMPCSolution(3 + temp_idx);
    //     controller_data_array[idx_GRM_left + temp_idx] = GMmpc.GetGMMPCSolution(6 + temp_idx);
    //     controller_data_array[idx_GRM_right + temp_idx] = GMmpc.GetGMMPCSolution(9 + temp_idx);

        // controller_data_array[idx_ref_com_rpy + temp_idx] = GMmpc.trajAll[12*(GMmpc.iterationCounter%5)+temp_idx];
        // controller_data_array[idx_ref_com_omega + temp_idx] = GMmpc.trajAll[12*(GMmpc.iterationCounter%5) + 6 + temp_idx];
        // controller_data_array[idx_ref_com_pos + temp_idx] = GMmpc.trajAll[12*(GMmpc.iterationCounter%5) + 3 + temp_idx];
        // controller_data_array[idx_ref_com_vel + temp_idx] = GMmpc.trajAll[12*(GMmpc.iterationCounter%5) + 9 + temp_idx];

        // controller_data_array[idx_ref_com_rpy + temp_idx] = GMmpc.Ref_Traj(0 + temp_idx, 0);
        // controller_data_array[idx_ref_com_pos + temp_idx] = GMmpc.Ref_Traj(9 + temp_idx, 0);
        // controller_data_array[idx_ref_com_omega + temp_idx] = GMmpc.Ref_Traj(18 + temp_idx, GMmpc.iterationCounter % 5);
        // controller_data_array[idx_ref_com_vel + temp_idx] = GMmpc.Ref_Traj(27 + temp_idx, GMmpc.iterationCounter % 5);

        // controller_data_array[idx_ref_foot_left_contact] = GMmpc.contact_states(0);
        // controller_data_array[idx_ref_foot_right_contact] = GMmpc.contact_states(1);
    //     if (GMmpc.contact_states(0) == 1)
    //     {
    //         controller_data_array[idx_ref_foot_left_pos + temp_idx] = 0;
    //         controller_data_array[idx_ref_foot_left_vel + temp_idx] = 0;
    //         controller_data_array[idx_ref_foot_right_pos + temp_idx] = GMmpc.pDesFootWorld(temp_idx); // should be checked
    //         controller_data_array[idx_ref_foot_right_vel + temp_idx] = GMmpc.vDesFootWorld(temp_idx);
    //     }
    //     else if (GMmpc.contact_states(1) == 1)
    //     {
    //         controller_data_array[idx_ref_foot_left_pos + temp_idx] = GMmpc.pDesFootWorld(temp_idx);
    //         controller_data_array[idx_ref_foot_left_vel + temp_idx] = GMmpc.vDesFootWorld(temp_idx);
    //         controller_data_array[idx_ref_foot_right_pos + temp_idx] = 0;
    //         controller_data_array[idx_ref_foot_right_vel + temp_idx] = 0;
    //     }
    // }
    for (int temp_idx = 0; temp_idx < 5; temp_idx++)
    {
        controller_data_array[idx_joint_cmd_tau + temp_idx] = _data->_legController->commands[0].tau(temp_idx);
        controller_data_array[idx_joint_cmd_tau + 5 + temp_idx] = _data->_legController->commands[1].tau(temp_idx);
        controller_data_array[idx_joint_cmd_pos + temp_idx] = _data->_legController->commands[0].qDes(temp_idx);
        controller_data_array[idx_joint_cmd_pos + 5 + temp_idx] = _data->_legController->commands[1].qDes(temp_idx);
        controller_data_array[idx_joint_cmd_vel + temp_idx] = _data->_legController->commands[0].qdDes(temp_idx);
        controller_data_array[idx_joint_cmd_vel + 5 + temp_idx] = _data->_legController->commands[1].qdDes(temp_idx);
    }
    // std::cout<<"controller_data_array debugging "<<idx_joint_cmd_pos<<" "<<_data->_legController->commands[0].qDes(3)<<std::endl;

    // controller_data_array[idx_ctrl_solver_time] = 0.001 * GMmpc.gmmpcsolvetime; // mpcsolvetime is in ms
    // controller_data_array[idx_ctrl_solver_cost] = GMmpc.t2; //TBA
    // controller_data_array[idx_ctrl_solver_iter] = get_solver_iter();


    // LIDAR added=======================================================

    // LIDAR added=======================================================

    for (int idx = 0; idx < sensor_data_size; idx++)
    {
        sensor_data << sensor_data_array[idx] << "  ";
    }
    for (int idx = 0; idx < estimated_data_size; idx++)
    {
        estimated_data << estimated_data_array[idx] << "  ";
    }
    for (int idx = 0; idx < controller_data_size; idx++)
    {
        controller_data << controller_data_array[idx] << "  ";
    }

    sensor_data << std::endl;
    estimated_data << std::endl;
    controller_data << std::endl;
    
}



void FSMState::CheckJointSafety(){
    


    for (int leg = 0; leg < 2; leg++){

        // Hip Constraint
        if ((_data->_legController->data[leg].q(0) < _data->_biped->Abad_Leg_Constraint[0]) || 
          (_data->_legController->data[leg].q(0) > _data->_biped->Abad_Leg_Constraint[1])) {
            std::cout << "Abad R Angle Exceeded" << _data->_legController->data[0].q(0) << std::endl;
            abort();
        }

        // AbAd Constraint
        if ((_data->_legController->data[leg].q(1) < _data->_biped->Hip_Leg_Constraint[0]) ||
            (_data->_legController->data[leg].q(1) > _data->_biped->Hip_Leg_Constraint[1])) {
            std::cout << "Hip R Angle Exceeded" << std::endl;
            abort();
        }

        //Thigh Constraint
        if ((_data->_legController->data[leg].q(2) < _data->_biped->Thigh_Constraint[0]) || 
        (_data->_legController->data[leg].q(2) > _data->_biped->Thigh_Constraint[1])) {
            std::cout << "Thigh Angle Exceeded" << std::endl;
            abort();
        }

        //Calf Constraint
        if ((_data->_legController->data[leg].q(3) < _data->_biped->Calf_Constraint[0]) || 
        (_data->_legController->data[leg].q(3) > _data->_biped->Calf_Constraint[1])) {
            std::cout << "Calf Angle Exceeded" << std::endl;
            abort();
        }

        //Ankle Constraint
        if ((_data->_legController->data[leg].q(4) < _data->_biped->Ankle_Constraint[0]) || 
        (_data->_legController->data[leg].q(4) > _data->_biped->Ankle_Constraint[1])) {
            std::cout << "Ankle Angle Exceeded" << std::endl;
            abort();
        }

    }


    //Pitch Constraint
    if ((_data->_stateEstimator->getResult().rpy(1)) < -0.3){
        std::cout << "Pitch Angle Exceeded" << std::endl;
        abort();
    }

}