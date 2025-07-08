#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include "cppTypes.h"
#include "RL.h"

class Biped{
  public:
    void setBiped(int real_flag = 1){ // 1 for Hardware, 0 for Sim

        _real_flag = real_flag;
        mu = 0.3; // friction coefficient
        f_max = 500; // maximum grf_z
        mass = 13.856;
        I_body << 0.5413, 0.0, 0.0, 0.0, 0.5200, 0.0, 0.0, 0.0, 0.0691;
        
        if (real_flag == 0)
        {
            // motor id mapping
            int temp_sequence[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
            for (int i = 0; i < 10; ++i) {
                motor_sequence[i] = temp_sequence[i];
            }

        }else{
            // motor id mapping
            int temp_sequence[] = {4, 5, 6, 7, 8, 1, 2, 9, 10, 11}; // left-right leg order
            for (int i = 0; i < 10; ++i) {
                motor_sequence[i] = temp_sequence[i];
            }
            offset = Joint_Calibration();
        }

        define_kinematics_coordinates();

    }

    // ** setter methods **

    void setFootHeight(double _foot_height){
        foot_height = _foot_height;
    }

    void setSteppingFrequency(double _gait_stepping_frequency){
        gait_stepping_frequency = _gait_stepping_frequency;
    }

    void setFrictionCoefficient(float _mu){
        mu = _mu;
    }

    void setSwingFootControlPoint(double _cp1_coef, double _cp2_coef){
        cp1_coef = _cp1_coef;
        cp2_coef = _cp2_coef;
    }

    void setFootPlacementZ(double _pf_z){
        pf_z = _pf_z;
    }

    void updateGaitParameter(Vec2<int> _dsp_durations, Vec2<int> _ssp_durations){
        dsp_durations = _dsp_durations;
        ssp_durations = _ssp_durations;
        reset_gait = true;
    }

    void updateSlope(double _slope_pitch){
        slope_pitch = _slope_pitch;
    }

    void setFootPlacementPlanner(std::string _foot_placement_planner){
        if (_foot_placement_planner == "LIP"){
            std::cout << "Foot placement planner is set to " << _foot_placement_planner << std::endl;
            foot_placement_planner = _foot_placement_planner;
        }
        else if (_foot_placement_planner == "Raibert"){
            std::cout << "Foot placement planner is set to " << _foot_placement_planner << std::endl;
            foot_placement_planner = _foot_placement_planner;
        }
        else{
            std::cout << "invalid planner choice, use default Raibert footplacement planner" << std::endl;
            foot_placement_planner = "Raibert";
        }
    }

    void update_mpc_cost(double _mpc_cost){
        mpc_cost = _mpc_cost;
    }

    // RL
    RL rl_params; // RL parameters

    //robot geometry
    double hipLinkLength, thighLinkLength, calfLinkLength;
    double leg_yaw_offset_x, leg_yaw_offset_y, leg_yaw_offset_z;
    double leg_roll_offset_x, leg_roll_offset_y, leg_roll_offset_z;
    Vec3<double> L_hipYawLocation;
    Vec3<double> L_hipRollLocation;
    Vec3<double> R_hipYawLocation;
    Vec3<double> R_hipRollLocation;


    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;

    // qp cost
    double mpc_cost = 0; 

    //robot dynamics
    float mass; 
    Eigen::Matrix<float, 3, 3> I_body; // inertial matrix about COM in body frame

    // other parameters
    float mu; 
    float f_max;
    float torque_limit[10]{33.5, 33.5, 33.5, 67.0, 33.5, 33.5, 33.5, 33.5, 67.0, 33.5};

    // parameters for reference and swing leg controller
    double foot_height=0.12; // swing foot height
    float gait_stepping_frequency = 1.0; // gait stepping frequency
    double cp1_coef = 0.333;
    double cp2_coef = 0.666;
    double pf_z = 0.0; // foot placement z value

    // Parameters for slope terrain 
    double slope_pitch = 0.0; // slope pitch in radian

    // foot placement planner (pick from here)
    std::string foot_placement_planner = "LIP";
    // std::string foot_placement_planner = "Raibert";
    // std::string foot_placement_planner = "OpenLoop";

    int robot_index; // 1 for Aliengo, 2 for A1
    int _real_flag = 1;

    // gait parameters
    bool reset_gait = false;
    Vec2<int> dsp_durations = {0, 0}; 
    Vec2<int> ssp_durations = {int(0.2/0.001), int(0.2/0.001)};


    double* offset;
    int motor_sequence[10];
    double gear_ratio = 1.545;
    double beltCompRatio = 1;
    double JointPDSwitch = 1.0;







    //Angle Constraints for Safety
    double Abad_Leg_Constraint[2] = {-30 * (3.1415/180), 30 * (3.1415/180)};
    double Hip_Leg_Constraint[2] = {-25* (3.1415/180.0), 55 * (3.1415/180)};
    // double Hip_Leg_Constraint[2] = {-35* (3.1415/180.0), 55 * (3.1415/180)};
    double Thigh_Constraint[2] = {0 * (3.1415/180), 80 * (3.1415/180)};
    double Calf_Constraint[2] = {-150 * (3.1415/180), -30 * (3.1415/180)};
    double Ankle_Constraint[2] = {-10 * (3.1415/180), 95 * (3.1415/180)};



    // ** kinematics code **
    // offset between each links in 0 positions (from URDF)
    double thigh_length = 0.22;
    double calf_length = 0.22;
    // double d_foot = 0.04; // original foot
    double d_foot = 0.0418; // capstone project foot
    Vec3<double> p1{-0.00, 0.047, -0.1265}; // base to hip yaw in frame1
    Vec3<double> p2{0.0465, 0.015, -0.0705}; // hip yaw to hip roll in frame2
    Vec3<double> p3{-0.06, 0.018, 0.0}; // hip roll to hip pitch in frame3
    Vec3<double> p4{0.0, 0.01805, -0.22}; // hip pitch to knee pitch in frame4
    Vec3<double> p5{0.0, 0.0, -0.22}; // knee pitch to angle pitch in frame5
    // Vec4<double> p5e{0.0, 0.0, -0.04, 1.0}; // ankle pitch to foot sole (original foot)
    Vec4<double> p5e{0.0, 0.0, -0.0418, 1.0}; // ankle pitch to foot sole (capstone project foot)
    Vec3<double> z1, z2, z3, z4, z5;

    Mat4<double> T01_left, T12p_left, T2p2_left, T23p_left, T3p3_left, T34_left, T45_left; 
    Mat4<double> T01_right, T12p_right, T2p2_right, T23p_right, T3p3_right, T34_right, T45_right;
    Mat4<double> T02_left, T03_left, T04_left, T05_left;
    Mat4<double> T02_right, T03_right, T04_right, T05_right;
    Vec3<double> p0e_left, p0e_right; // end-effector position in body frame
    Mat65<double> J_left, J_right; // contact jacobian
    Vec3<double> p01_left, p02_left, p03_left, p04_left, p05_left;
    Vec3<double> p01_right, p02_right, p03_right, p04_right, p05_right;

    Vec3<double> get_hip_yaw_offset(int leg)
    {
        if (leg == 0)
            return p01_left;
        else
            return p01_right;
    }

    Vec3<double> get_hip_offset(int leg)
    {
        if (leg == 0) {
            Vec3<double> offset = {-0.00+0.0465-0.06, 0.047+0.015+0.03605, 0.0};
            return offset;
        }
        else{
            Vec3<double> offset = {-0.00+0.0465-0.06, -0.047-0.015-0.03605, 0.0};
            return offset;
        }
    }

    Mat3<double> rot_x(double angle)
    {
        Mat3<double> R;
        R << 1, 0,          0,
             0, cos(angle), -sin(angle),
             0, sin(angle),  cos(angle);
        return R;
    }

    Mat3<double> rot_y(double angle)
    {
        Mat3<double> R;
        R << cos(angle), 0, sin(angle),
             0,          1, 0,
            -sin(angle), 0, cos(angle);
        return R;
    }

    Mat3<double> rot_z(double angle)
    {
        Mat3<double> R;
        R << cos(angle), -sin(angle), 0,
             sin(angle),  cos(angle), 0,
             0,          0,          1;
        return R;
    }

    void define_kinematics_coordinates()
    {
        T01_left.setIdentity();
        T2p2_left.setIdentity();
        T3p3_left.setIdentity();
        T34_left.setIdentity();
        T45_left.setIdentity();
        T12p_left << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
        T23p_left << 0, 1, 0, 0,
                0, 0, 1, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;

        T01_right.setIdentity();
        T2p2_right.setIdentity();
        T3p3_right.setIdentity();
        T34_right.setIdentity();
        T45_right.setIdentity();
        T12p_right << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;

        T23p_right << 0, 1, 0, 0,
                0, 0, 1, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
        
        // offset vectors
        p2 = T12p_left.block<3,3>(0,0).transpose() * p2;
        p3 = T23p_left.block<3,3>(0,0).transpose() * T12p_left.block<3,3>(0,0).transpose() * p3;
        p4 = T23p_left.block<3,3>(0,0).transpose() * T12p_left.block<3,3>(0,0).transpose() * p4;
        p5 = T23p_left.block<3,3>(0,0).transpose() * T12p_left.block<3,3>(0,0).transpose() * p5; 
        p5e = T23p_left.transpose() * T12p_left.transpose() * p5e;
    }

    void forward_kinematics(Vec5<double> &joint_angles, int leg){
        if (leg == 0){
            T45_left.block<3,3>(0,0) = rot_z(joint_angles(4));
            T45_left.block<3,1>(0,3) = p5;
            T34_left.block<3,3>(0,0) = rot_z(joint_angles(3));
            T34_left.block<3,1>(0,3) = p4;
            T3p3_left.block<3,3>(0,0) = rot_z(joint_angles(2));
            T3p3_left.block<3,1>(0,3) = p3;
            T2p2_left.block<3,3>(0,0) = rot_z(joint_angles(1));
            T2p2_left.block<3,1>(0,3) = p2;
            T01_left.block<3,3>(0,0) = rot_z(joint_angles(0));
            T01_left.block<3,1>(0,3) = p1;

            T02_left = T01_left * T12p_left * T2p2_left; 
            T03_left = T02_left * T23p_left * T3p3_left;
            T04_left = T03_left * T34_left;
            T05_left = T04_left * T45_left;

            p0e_left = (T05_left * p5e).head<3>(); 

        }
        else if (leg == 1){
            Vec3<double> side_vec_1 = {1.0, 1.0, -1.0};
            Vec3<double> side_vec_2 = {1.0, -1.0, 1.0};
            T45_right.block<3,3>(0,0) = rot_z(joint_angles(4));
            T45_right.block<3,1>(0,3) = p5.cwiseProduct(side_vec_1);
            T34_right.block<3,3>(0,0) = rot_z(joint_angles(3));
            T34_right.block<3,1>(0,3) = p4.cwiseProduct(side_vec_1);
            T3p3_right.block<3,3>(0,0) = rot_z(joint_angles(2));
            T3p3_right.block<3,1>(0,3) = p3.cwiseProduct(side_vec_1);
            T2p2_right.block<3,3>(0,0) = rot_z(joint_angles(1));
            T2p2_right.block<3,1>(0,3) = p2.cwiseProduct(side_vec_2);
            T01_right.block<3,3>(0,0) = rot_z(joint_angles(0));
            T01_right.block<3,1>(0,3) = p1.cwiseProduct(side_vec_2);

            T02_right = T01_right * T12p_right * T2p2_right; 
            T03_right = T02_right * T23p_right * T3p3_right;
            T04_right = T03_right * T34_right;
            T05_right = T04_right * T45_right;

            p0e_right = (T05_right * p5e).head<3>();

        }
    }

    Vec3<double> get_p0e(int leg)
    {
        if (leg == 0){
            return p0e_left;
        }
        else{
            return p0e_right;
        }
    }


    void contact_jacobian(int leg)
    {
        if (leg == 0){
            Vec3<double> z_axis = {0.0, 0.0, 1.0};
            p01_left = T01_left.block<3,1>(0,3);
            p02_left = T02_left.block<3,1>(0,3);
            p03_left = T03_left.block<3,1>(0,3);
            p04_left = T04_left.block<3,1>(0,3);
            p05_left = T05_left.block<3,1>(0,3);

            z1 = T01_left.block<3,3>(0,0) * z_axis;
            z2 = T02_left.block<3,3>(0,0) * z_axis;
            z3 = T03_left.block<3,3>(0,0) * z_axis;
            z4 = T04_left.block<3,3>(0,0) * z_axis;
            z5 = T05_left.block<3,3>(0,0) * z_axis;

            J_left.block<3,1>(0, 0) = z1.cross(p0e_left - p01_left);
            J_left.block<3,1>(3, 0) = z1;
            J_left.block<3,1>(0, 1) = z2.cross(p0e_left - p02_left);
            J_left.block<3,1>(3, 1) = z2;
            J_left.block<3,1>(0, 2) = z3.cross(p0e_left - p03_left);
            J_left.block<3,1>(3, 2) = z3;
            J_left.block<3,1>(0, 3) = z4.cross(p0e_left - p04_left);
            J_left.block<3,1>(3, 3) = z4;
            J_left.block<3,1>(0, 4) = z5.cross(p0e_left - p05_left);
            J_left.block<3,1>(3, 4) = z5;
        }

        else if (leg == 1){
            Vec3<double> z_axis = {0.0, 0.0, 1.0};
            p01_right = T01_right.block<3,1>(0,3);
            p02_right = T02_right.block<3,1>(0,3);
            p03_right = T03_right.block<3,1>(0,3);
            p04_right = T04_right.block<3,1>(0,3);
            p05_right = T05_right.block<3,1>(0,3);

            z1 = T01_right.block<3,3>(0,0) * z_axis;
            z2 = T02_right.block<3,3>(0,0) * z_axis;
            z3 = T03_right.block<3,3>(0,0) * z_axis;
            z4 = T04_right.block<3,3>(0,0) * z_axis;
            z5 = T05_right.block<3,3>(0,0) * z_axis;

            J_right.block<3,1>(0, 0) = z1.cross(p0e_right - p01_right);
            J_right.block<3,1>(3, 0) = z1;
            J_right.block<3,1>(0, 1) = z2.cross(p0e_right - p02_right);
            J_right.block<3,1>(3, 1) = z2;
            J_right.block<3,1>(0, 2) = z3.cross(p0e_right - p03_right);
            J_right.block<3,1>(3, 2) = z3;
            J_right.block<3,1>(0, 3) = z4.cross(p0e_right - p04_right);
            J_right.block<3,1>(3, 3) = z4;
            J_right.block<3,1>(0, 4) = z5.cross(p0e_right - p05_right);
            J_right.block<3,1>(3, 4) = z5;
        }
    }

    Mat65<double> get_contact_jacobian(int leg)
    {
        if (leg == 0){
            return J_left;
        }
        else{
            return J_right;
        }
    }

    Vec3<double> analytical_IK(Vec3<double> &p_foot_des_b, int leg)
    {
        // Analytic IK code (track only foot positin, not orientation)
        // Arguments: 
        // p_foot_des_b: desired foot position in body frame
        // leg: 0 for left, 1 for right
        // Returns: joint angles for hip roll, hip pitch, knee pitch

        Vec3<double> q;

        double side; 
        if (leg == 0) { // left
            side = -1.0;
        }
        else if (leg == 1) { // right
            side = 1.0;
        }

        Eigen::Vector3d hip_roll = {-0.00+0.0465-0.06, -0.047*side-0.015*side, -0.1265-0.0705}; // hip roll origin in body frame
        Eigen::Vector3d foot_des_from_hip_roll = p_foot_des_b - hip_roll; // foot target position in hip roll frame (orientation aligned with body frame)
        foot_des_from_hip_roll(2) += d_foot; // track ankle joint position instead of sole position
        
        // analytical IK solutions
        double distance_2D_yOz = std::sqrt(std::pow(foot_des_from_hip_roll[1], 2) + std::pow(foot_des_from_hip_roll[2], 2)); // r1_yz
        double distance_horizontal = 0.018+0.01805; // hip roll to end-effector y distance

        // hip roll
        q(0) = std::asin(clamp(foot_des_from_hip_roll[1] / distance_2D_yOz, -1.0, 1.0)) + std::asin(clamp(distance_horizontal * side / distance_2D_yOz, -1.0, 1.0));

        // transform foot_des_from_hip_roll to hip pitch frame
        Mat3<double> R_hip_roll = rot_x(q(0));
        Vec3<double> hip_roll_to_hip_pitch_offset = {0.0, 0.018*side, 0.0};
        Vec3<double> foot_des_from_hip_pitch = R_hip_roll.transpose() * foot_des_from_hip_roll + hip_roll_to_hip_pitch_offset;
        double r = foot_des_from_hip_pitch.norm();

        // planar 2R IK (xz plane)
        double cos_q2 = clamp((std::pow(r, 2) - std::pow(thigh_length, 2) - std::pow(calf_length, 2)) / (2.0 * thigh_length * calf_length), -1.0, 1.0); 
        double sin_q2 = clamp(-std::sqrt(1.0 - std::pow(cos_q2, 2)), -1.0, 1.0);
        q(2) = std::atan2(sin_q2, cos_q2); 
        q(1) = std::atan2(-foot_des_from_hip_pitch(0), -foot_des_from_hip_pitch(2)) - std::atan2(calf_length * sin_q2, thigh_length + calf_length * cos_q2);

        return q;
    }

    double clamp(double val, double minVal, double maxVal) {
        return std::max(minVal, std::min(val, maxVal));
    }



    // **** The following is interface code ****
    // ** joint - motor interface code **

    double* Joint_Calibration(){
        std::ifstream angle_file;
        std::string angle_name;
        int i = 0;

        angle_file.open("../Interface/HW_interface/Calibration/offset.txt");

        getline(angle_file, angle_name);
        std::cout << "Calibration Done." << std::endl;

        std::stringstream ss(angle_name);
        double angle1, angle2, angle3, angle4, angle5, 
                angle6, angle7, angle8, angle9, angle10;
        ss >> angle1 >> angle2 >> angle3 >> angle4 >> angle5 >> angle6 >> angle7 >> angle8 >> angle9 >> angle10;

        static double offset_angle[10] = {angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9, angle10};

        return offset_angle;
    }

    // raw encoder reading to joint angle
    void Joint_Remapping_lowfromhigh(int leg, Vec5<double> &q, Vec5<double> &qd, Vec5<double> &tau){

        if (_real_flag == 1){

            // offset importing
            for (int i = 0; i < 5; i++)
            {
                q[i] = q[i] + offset[i + leg * 5];
            }

            // Unitree-related calibration
            if (leg == 0)
            {
                q(0) = -q(0);
                q(1) = -q(1);
                q(2) = -q(2);
                qd(0) = -qd(0);
                qd(1) = -qd(1);
                qd(2) = -qd(2);
            }

            // For knee pully system
            q(3) = q(3) / gear_ratio;
            qd(3) = qd(3) / gear_ratio;
            tau(3) = tau(3) * gear_ratio * beltCompRatio;

            // For Ankle actuation linkage
            q(4) = q(4) - q(3);
            qd(4) = qd(4) - qd(3);
        
        }

    }



    // joint angle to raw encoder reading
    void Joint_Remapping_highfromlow(int leg, Vec5<double> &q, Vec5<double> &qd, Vec5<double> &tau){

        if (_real_flag == 1){

            // For Ankle actuation linkage and knee pully system
            q[4] = q[4] + q[3];
            qd[4] = qd[4] + qd[3];

            q[3] = q[3] * gear_ratio;
            qd[3] = qd[3] * gear_ratio;
            tau[3] = tau[3] / gear_ratio * beltCompRatio;

            // Unitree-related
            if (leg == 0) {
                q[0]   *= -1.0;
                q[1]   *= -1.0;
                q[2]   *= -1.0;

                qd[0]   *= -1.0;
                qd[1]   *= -1.0;
                qd[2]   *= -1.0;

                tau[0]   *= -1.0;
                tau[1]   *= -1.0;
                tau[2]   *= -1.0;
            }

            //offset 
            for(int i = 0; i < 5; i++){
                q[i] -= offset[i+leg*5];
            }

        
        }

    }

    bool CheckMotorConnection(int leg, Vec5<double> data_q){

        if (_real_flag == 1){
            // check motor connection
            for (int j = 0; j < 5; j++)
            {
                if (data_q[j] == 0)
                {
                    std::cout << "\nMotor Connection on leg " << leg << " at " << j << " LOST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                    return false;
                }
            }
        }
        return true;

    }

    void SafeGuardUnusedMotor(LowlevelCmd* cmd){
        //For unused actuators - just for safety
        if (_real_flag == 1){
            cmd->motorCmd[0].tau = 0;
            cmd->motorCmd[3].tau = 0;
            cmd->motorCmd[0].Kp = 0;
            cmd->motorCmd[0].Kd = 5;
            cmd->motorCmd[3].Kp = 0;
            cmd->motorCmd[3].Kd = 5;
        }

        else {
            cmd->motorCmd[10].tau = 0;
            cmd->motorCmd[11].tau = 0;
            cmd->motorCmd[10].Kp = 0;
            cmd->motorCmd[11].Kd = 5;
            cmd->motorCmd[10].Kp = 0;
            cmd->motorCmd[11].Kd = 5;
        }
    }

    Vec3<double> getHipYawLocation(int leg){
        return Vec3<double>(leg_yaw_offset_x, leg == 0 ? leg_yaw_offset_y : -leg_yaw_offset_y, leg_yaw_offset_z);
    }

    Vec3<double> getHipRollLocation(int leg){
        return Vec3<double>(leg_roll_offset_x, leg == 0 ? leg_roll_offset_y : -leg_roll_offset_y, leg_roll_offset_z);
    }


};

#endif
