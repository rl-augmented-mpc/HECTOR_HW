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

void FSMState::Logging(ConvexMPCLocomotion &mpc)
{

    // Data Recording
    // sensor data recording
    for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    {
        sensor_data_array[idx_imu + temp_idx] = _data->_stateEstimator->getResult().omegaWorld(temp_idx);
        sensor_data_array[idx_imu + 3 + temp_idx] = _data->_stateEstimator->getResult().aWorld(temp_idx);
    }
    for (int temp_idx = 0; temp_idx < 5; temp_idx++)
    {
        sensor_data_array[idx_joint_pos + temp_idx] = _data->_legController->data[0].q(temp_idx);       // left
        sensor_data_array[idx_joint_pos + 5 + temp_idx] = _data->_legController->data[1].q(temp_idx);   // right
        sensor_data_array[idx_joint_vel + temp_idx] = _data->_legController->data[0].qd(temp_idx);      // left
        sensor_data_array[idx_joint_vel + 5 + temp_idx] = _data->_legController->data[1].qd(temp_idx);  // right
        sensor_data_array[idx_joint_tau + temp_idx] = _data->_legController->data[0].tau(temp_idx);     // left
        sensor_data_array[idx_joint_tau + 5 + temp_idx] = _data->_legController->data[1].tau(temp_idx); // right
    }
    for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    {
        sensor_data_array[idx_foot_pos + temp_idx] = _data->_legController->data[0].p(temp_idx); // left
        sensor_data_array[idx_foot_pos + 3 + temp_idx] = _data->_legController->data[1].p(temp_idx); // right
        sensor_data_array[idx_foot_vel + temp_idx] = _data->_legController->data[0].v(temp_idx); // left
        sensor_data_array[idx_foot_vel + 3 + temp_idx] = _data->_legController->data[1].v(temp_idx); // right
    }

    // state estimator data recording
    for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    {
        estimated_data_array[idx_com_rpy + temp_idx] = _data->_stateEstimator->getResult().rpy(temp_idx);
        estimated_data_array[idx_com_omega + temp_idx] = _data->_stateEstimator->getResult().omegaWorld(temp_idx);
        estimated_data_array[idx_com_pos + temp_idx] = _data->_stateEstimator->getResult().position(temp_idx); // or T265_pose
        estimated_data_array[idx_com_vel + temp_idx] = _data->_stateEstimator->getResult().vWorld(temp_idx);
        estimated_data_array[idx_com_acc + temp_idx] = _data->_stateEstimator->getResult().aWorld(temp_idx);
    }

    estimated_data_array[idx_foot_left_contact] = _data->_legController->commands[0].contact_state; 
    estimated_data_array[idx_foot_right_contact] = _data->_legController->commands[1].contact_state;


    // controller data recording
    for (int temp_idx = 0; temp_idx < 3; temp_idx++){
        controller_data_array[idx_GRF_left + temp_idx] = _data->_legController->commands[0].feedforwardForce(temp_idx);
        controller_data_array[idx_GRF_right + temp_idx] = _data->_legController->commands[1].feedforwardForce(temp_idx);
        controller_data_array[idx_GRM_left + temp_idx] = _data->_legController->commands[0].feedforwardForce(temp_idx+3);
        controller_data_array[idx_GRM_right + temp_idx] = _data->_legController->commands[1].feedforwardForce(temp_idx+3);
    }
    
    for (int temp_idx = 0; temp_idx < 5; temp_idx++)
    {
        controller_data_array[idx_joint_cmd_tau + temp_idx] = _data->_legController->commands[0].tau(temp_idx);
        controller_data_array[idx_joint_cmd_tau + 5 + temp_idx] = _data->_legController->commands[1].tau(temp_idx);
        controller_data_array[idx_joint_cmd_pos + temp_idx] = _data->_legController->commands[0].qDes(temp_idx);
        controller_data_array[idx_joint_cmd_pos + 5 + temp_idx] = _data->_legController->commands[1].qDes(temp_idx);
        controller_data_array[idx_joint_cmd_vel + temp_idx] = _data->_legController->commands[0].qdDes(temp_idx);
        controller_data_array[idx_joint_cmd_vel + 5 + temp_idx] = _data->_legController->commands[1].qdDes(temp_idx);
    }

    for (int temp_idx = 0; temp_idx < 3; temp_idx++)
    {   
        if (temp_idx == 0){
            controller_data_array[idx_ref_com_rpy + temp_idx] = 0.0;
            controller_data_array[idx_ref_com_omega + temp_idx] = 0.0;
        }

        else if (temp_idx == 1){
            controller_data_array[idx_ref_com_rpy + temp_idx] = 0.0;
            controller_data_array[idx_ref_com_omega + temp_idx] = 0.0;
        }
        else if (temp_idx == 2){
            controller_data_array[idx_ref_com_rpy + temp_idx] = mpc.yaw_desired; 
            controller_data_array[idx_ref_com_omega + temp_idx] = mpc.turn_rate_des;
        }

        controller_data_array[idx_ref_com_pos + temp_idx] = mpc.world_position_desired(temp_idx);
        controller_data_array[idx_ref_com_vel + temp_idx] = mpc.v_des_world(temp_idx);
    }

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

    if (_data->_biped->_real_flag == 1){
        for (int leg = 0; leg < 2; leg++){

            // Hip Constraint
            if ((_data->_legController->data[leg].q(0) < _data->_biped->Abad_Leg_Constraint[0]) || 
            (_data->_legController->data[leg].q(0) > _data->_biped->Abad_Leg_Constraint[1])) {
                std::cout << "leg: " << leg << " Hip Yaw Angle Exceeded " << _data->_legController->data[leg].q(0) << std::endl;
                abort();
            }

            // AbAd Constraint
            if ((_data->_legController->data[leg].q(1) < _data->_biped->Hip_Leg_Constraint[0]) ||
                (_data->_legController->data[leg].q(1) > _data->_biped->Hip_Leg_Constraint[1])) {
                std::cout << "leg: " << leg << " Hip Roll Angle Exceeded " << _data->_legController->data[leg].q(1) << std::endl;
                abort();
            }

            //Thigh Constraint
            if ((_data->_legController->data[leg].q(2) < _data->_biped->Thigh_Constraint[0]) || 
            (_data->_legController->data[leg].q(2) > _data->_biped->Thigh_Constraint[1])) {
                std::cout << "leg: " << leg << " Thigh Angle Exceeded " << _data->_legController->data[leg].q(2) << std::endl;
                abort();
            }

            //Calf Constraint
            if ((_data->_legController->data[leg].q(3) < _data->_biped->Calf_Constraint[0]) || 
            (_data->_legController->data[leg].q(3) > _data->_biped->Calf_Constraint[1])) {
                std::cout << "leg: " << leg << " Calf Angle Exceeded " << _data->_legController->data[leg].q(3) << std::endl;
                abort();
            }

            //Ankle Constraint
            if ((_data->_legController->data[leg].q(4) < _data->_biped->Ankle_Constraint[0]) || 
            (_data->_legController->data[leg].q(4) > _data->_biped->Ankle_Constraint[1])) {
                std::cout << "leg: " << leg << " Ankle Angle Exceeded " << _data->_legController->data[leg].q(4) << std::endl;
                abort();
            }

        }


        // Body Pitch Constraint
        if ((_data->_stateEstimator->getResult().rpy(1)) < -0.3){
            std::cout << "Body Pitch Angle Exceeded: " << 180/3.141 * _data->_stateEstimator->getResult().rpy(1) << std::endl;
            abort();
        }
    }
}