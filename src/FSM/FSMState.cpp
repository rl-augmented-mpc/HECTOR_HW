#include "../../include/FSM/FSMState.h"

// float FSMState::T265_pose[6] = {0, 0, 0, 0, 0, 0};
// bool FSMState::rs2_initialized = false;
// rs2::pipeline FSMState::pipe;

FSMState::FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr):
            _data(data), _stateName(stateName), _stateNameStr(stateNameStr)
{
    _lowCmd = _data->_lowCmd;
    _lowState = _data->_lowState;
    
    myfile.open("ori.txt");
    QP.open("QPsolution.txt");
    com_pos.open("COMpos.txt");
    b_des.open("b_des_z.txt");
    angle.open("angle.txt");
    torque.open("torque.txt");
    footposition.open("foot.txt");
    rpy_input.open("rpy_files.txt");
    force.open("force.txt");
    omega.open("omega.txt");
    acceleration.open("acceleration.txt");
    tau_est.open("Estimated_torque.txt");
    corrected_angle.open("corrected_angle.txt");
    T265_pos.open("T265_pos.txt");
    T265_qua.open("T265_quaternion.txt");
    fullStateTraj.open("fullState.txt");

    offset = Angle_Caliberation();

    // if (!rs2_initialized) {
    //     // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    //     // pipe.start(cfg);
    //     // rs2_initialized = true;
    //     // std::cout << "Initialized rs2" << std::endl;
    // }
}


double *FSMState::Angle_Caliberation(){
    std::ifstream angle_file;
    std::string angle_name;
    int i = 0;

    angle_file.open("../Calibration/offset.txt");

    getline(angle_file, angle_name);
    std::cout << "Angle string is " << angle_name << std::endl;

    std::stringstream ss(angle_name);
    double angle1, angle2, angle3, angle4, angle5, 
            angle6, angle7, angle8, angle9, angle10;
    ss >> angle1 >> angle2 >> angle3 >> angle4 >> angle5 >> angle6 >> angle7 >> angle8 >> angle9 >> angle10;

    static double offset_angle[10] = {angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9, angle10};
    std::cout << "Offset in function is " << angle1 << std::endl;

    return offset_angle;
}
