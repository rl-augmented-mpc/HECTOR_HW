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
    void setBiped(int real_flag = 1){ // 1 for Real, 0 for Sim

        _real_flag = real_flag;
        mu = 0.3; // friction coefficient
        f_max = 500; // maximum grf_z
        
        if (real_flag == 0)
        {
            mass = 13.856;
            I_body << 0.5413, 0.0, 0.0, 0.0, 0.5200, 0.0, 0.0, 0.0, 0.0691;

            leg_offset_x = -0.005;
            leg_offset_y = -0.047;
            leg_offset_z = -0.126;

            hipLinkLength = 0.038; // hip offset in const.xacro
            thighLinkLength = 0.22;
            calfLinkLength = 0.22;
             
            int temp_sequence[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
            for (int i = 0; i < 10; ++i) {
                motor_sequence[i] = temp_sequence[i];
            }

        }else{
            int temp_sequence[] = {4, 5, 6, 7, 8, 1, 2, 9, 10, 11}; // left-right leg
            // int temp_sequence[] = {1, 2, 9, 10, 11, 4, 5, 6, 7, 8}; // right-left leg
            for (int i = 0; i < 10; ++i) {
                motor_sequence[i] = temp_sequence[i];
            }

            offset = Joint_Calibration();

            mass = 13.856;
            I_body << 0.5413, 0.0, 0.0, 0.0, 0.5200, 0.0, 0.0, 0.0, 0.0691;

            leg_offset_x = 0.0;
            leg_offset_y = 0.047;
            leg_offset_z = -0.1265;


            hipLinkLength = 0.038; // hip offset in const.xacro
            thighLinkLength = 0.22;
            calfLinkLength = 0.22;

            // length of a vector from ankle to toe and lh is to heel
            lt = 0.07;
            lh = 0.04; 
            lw = 0.05;
            
            lt_vec << 0, 0, lt;
            lh_vec << 0, 0, lh;
            lw_vec << 0, 0, lw;
        }



    }

    void setFootHeight(double _foot_height){
        foot_height = _foot_height;
    }

    void setSteppingFrequency(double _gait_stepping_frequency){
        gait_stepping_frequency = _gait_stepping_frequency;
    }

    void setFrictionCoefficient(float _mu){
        mu = _mu;
    }

    // RL
    RL rl_params; // RL parameters

    //robot geometry
    double hipLinkLength;
    double thighLinkLength;
    double calfLinkLength;

    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;

    double foot_area;

    //robot dynamics
    float mass; 
    Eigen::Matrix<float, 3, 3> I_body; // inertial matrix about COM in body frame

    // other parameters
    float mu; 
    float f_max;
    float torque_limit[10]{33.5, 33.5, 33.5, 67.0, 33.5, 33.5, 33.5, 33.5, 67.0, 33.5};

    double m_feet[2];
    Eigen::Matrix<double,3,3> I_feet[2];

    double lt;
    double lh;
    double lw;

    Eigen::Matrix<double, 1, 3> lt_vec;
    Eigen::Matrix<double, 1, 3> lt_3D;
    Eigen::Matrix<double, 1, 3> lh_vec;
    Eigen::Matrix<double, 1, 3> lh_3D;
    Eigen::Matrix<double, 1, 3> lw_vec;

    // parameters for reference and swing leg controller
    double foot_height=0.12; // swing foot height
    float gait_stepping_frequency = 1.0; // gait stepping frequency

    // miscallenous
    int robot_index; // 1 for Aliengo, 2 for A1
    int _real_flag = 1;


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




    // encoder data remapping
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
        
        }else{

            double PI = 3.14159265359;
            q[2] += 0.3*PI; 
            q[3] -= 0.6*PI; 
            q[4] += 0.3*PI;

        }

    }



    // encoder data remapping
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

        
        }else{

            double PI = 3.14159265359;
            q[2] -= 0.3*PI; 
            q[3] += 0.6*PI; 
            q[4] -= 0.3*PI;

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



    // get com to hip offset
    Vec3<double> getHip2Location(int leg){
        assert(leg >=0 && leg <2);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0){
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1){
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }

        return pHip;
    };



    Vec3<double> HiptoFoot(Vec5<double> &joint_angles, int leg)
    {
        Vec3<double> Hip2Foot;
        double side;
        double joint0_angle = joint_angles(0);
        double joint1_angle = joint_angles(1);
        double joint2_angle = joint_angles(2);
        double joint3_angle = joint_angles(3);
        double joint4_angle;

        if (_real_flag == 0){
            double joint4_angle = joint_angles(4);
        }
        else{
            double joint4_angle = joint_angles(4) + joint3_angle;
        }

        if (leg == 0){
            side = 1.0;
        }else{
            side = -1.0;
        }

        Hip2Foot(0) = -(3 * cos(joint0_angle)) / 200 - (9 * sin(joint4_angle) * (cos(joint3_angle) * (cos(joint0_angle) * cos(joint2_angle) - sin(joint0_angle) * sin(joint1_angle) * sin(joint2_angle)) - sin(joint3_angle) * (cos(joint0_angle) * sin(joint2_angle) + cos(joint2_angle) * sin(joint0_angle) * sin(joint1_angle)))) / 250 - (11 * cos(joint0_angle) * sin(joint2_angle)) / 50 - ((side)*sin(joint0_angle)) / 50 - (11 * cos(joint3_angle) * (cos(joint0_angle) * sin(joint2_angle) + cos(joint2_angle) * sin(joint0_angle) * sin(joint1_angle))) / 50 - (11 * sin(joint3_angle) * (cos(joint0_angle) * cos(joint2_angle) - sin(joint0_angle) * sin(joint1_angle) * sin(joint2_angle))) / 50 - (9 * cos(joint4_angle) * (cos(joint3_angle) * (cos(joint0_angle) * sin(joint2_angle) + cos(joint2_angle) * sin(joint0_angle) * sin(joint1_angle)) + sin(joint3_angle) * (cos(joint0_angle) * cos(joint2_angle) - sin(joint0_angle) * sin(joint1_angle) * sin(joint2_angle)))) / 250 - (23 * cos(joint1_angle) * (side)*sin(joint0_angle)) / 1000 - (11 * cos(joint2_angle) * sin(joint0_angle) * sin(joint1_angle)) / 50;
        Hip2Foot(1) = (cos(joint0_angle) * (side)) / 50 - (9 * sin(joint4_angle) * (cos(joint3_angle) * (cos(joint2_angle) * sin(joint0_angle) + cos(joint0_angle) * sin(joint1_angle) * sin(joint2_angle)) - sin(joint3_angle) * (sin(joint0_angle) * sin(joint2_angle) - cos(joint0_angle) * cos(joint2_angle) * sin(joint1_angle)))) / 250 - (3 * sin(joint0_angle)) / 200 - (11 * sin(joint0_angle) * sin(joint2_angle)) / 50 - (11 * cos(joint3_angle) * (sin(joint0_angle) * sin(joint2_angle) - cos(joint0_angle) * cos(joint2_angle) * sin(joint1_angle))) / 50 - (11 * sin(joint3_angle) * (cos(joint2_angle) * sin(joint0_angle) + cos(joint0_angle) * sin(joint1_angle) * sin(joint2_angle))) / 50 - (9 * cos(joint4_angle) * (cos(joint3_angle) * (sin(joint0_angle) * sin(joint2_angle) - cos(joint0_angle) * cos(joint2_angle) * sin(joint1_angle)) + sin(joint3_angle) * (cos(joint2_angle) * sin(joint0_angle) + cos(joint0_angle) * sin(joint1_angle) * sin(joint2_angle)))) / 250 + (23 * cos(joint0_angle) * cos(joint1_angle) * (side)) / 1000 + (11 * cos(joint0_angle) * cos(joint2_angle) * sin(joint1_angle)) / 50;
        Hip2Foot(2) = (23 * (side)*sin(joint1_angle)) / 1000 - (11 * cos(joint1_angle) * cos(joint2_angle)) / 50 - (9 * cos(joint4_angle) * (cos(joint1_angle) * cos(joint2_angle) * cos(joint3_angle) - cos(joint1_angle) * sin(joint2_angle) * sin(joint3_angle))) / 250 + (9 * sin(joint4_angle) * (cos(joint1_angle) * cos(joint2_angle) * sin(joint3_angle) + cos(joint1_angle) * cos(joint3_angle) * sin(joint2_angle))) / 250 - (11 * cos(joint1_angle) * cos(joint2_angle) * cos(joint3_angle)) / 50 + (11 * cos(joint1_angle) * sin(joint2_angle) * sin(joint3_angle)) / 50 - 3.0 / 50.0;

        return Hip2Foot;

    }




    // J is 65; 5 joints, 6 DOF
    //  0 for Sim, 1 for Real
    // TODO: verify sim version on hardware
    Mat65<double> HiptoFootJacobian(Vec5<double> &joint_angles, int leg)
    {
        Mat65<double> J;

        double side;

        if (_real_flag == 0) // Compute Leg Jacobian in Sim environment.
        {

            double q0 = joint_angles(0);
            double q1 = joint_angles(1);
            double q2 = joint_angles(2);
            double q3 = joint_angles(3);
            double q4 = joint_angles(4);


            if (leg == 0){
                side = 1.0;
            }else{
                side = -1.0;
            }


            J(0, 0) = sin(q0) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3) + 0.22 * sin(q2) + 0.0135) + cos(q0) * (0.015 * side + cos(q1) * (0.018 * side + 0.0025) - 1.0 * sin(q1) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2)));
            J(1, 0) = sin(q0) * (0.015 * side + cos(q1) * (0.018 * side + 0.0025) - 1.0 * sin(q1) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2))) - 1.0 * cos(q0) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3) + 0.22 * sin(q2) + 0.0135);
            J(2, 0) = 0.0;
            J(3, 0) = 0.0;
            J(4, 0) = 0.0;
            J(5, 0) = 1.0;

            J(0, 1) = -1.0 * sin(q0) * (sin(q1) * (0.018 * side + 0.0025) + cos(q1) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2)));
            J(1, 1) = cos(q0) * (sin(q1) * (0.018 * side + 0.0025) + cos(q1) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2)));
            J(2, 1) = sin(q1) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2)) - 1.0 * cos(q1) * (0.018 * side + 0.0025);
            J(3, 1) = cos(q0);
            J(4, 1) = sin(q0);
            J(5, 1) = 0.0;

            J(0, 2) = sin(q0) * sin(q1) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3) + 0.22 * sin(q2)) - 1.0 * cos(q0) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2));
            J(1, 2) = -1.0 * sin(q0) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3) + 0.22 * cos(q2)) - 1.0 * cos(q0) * sin(q1) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3) + 0.22 * sin(q2));
            J(2, 2) = cos(q1) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3) + 0.22 * sin(q2));
            J(3, 2) = -cos(q1) * sin(q0);
            J(4, 2) = cos(q0) * cos(q1);
            J(5, 2) = sin(q1);

            J(0, 3) = sin(q0) * sin(q1) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3)) - 1.0 * cos(q0) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3));
            J(1, 3) = -1.0 * sin(q0) * (0.04 * cos(q2 + q3 + q4) + 0.22 * cos(q2 + q3)) - 1.0 * cos(q0) * sin(q1) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3));
            J(2, 3) = cos(q1) * (0.04 * sin(q2 + q3 + q4) + 0.22 * sin(q2 + q3));
            J(3, 3) = -cos(q1) * sin(q0);
            J(4, 3) = cos(q0) * cos(q1);
            J(5, 3) = sin(q1);

            J(0, 4) = 0.04 * sin(q2 + q3 + q4) * sin(q0) * sin(q1) - 0.04 * cos(q2 + q3 + q4) * cos(q0);
            J(1, 4) = -0.04 * cos(q2 + q3 + q4) * sin(q0) - 0.04 * sin(q2 + q3 + q4) * cos(q0) * sin(q1);
            J(2, 4) = 0.04 * sin(q2 + q3 + q4) * cos(q1);
            J(3, 4) = -cos(q1) * sin(q0);
            J(4, 4) = cos(q0) * cos(q1);
            J(5, 4) = sin(q1);



        }else{

            double side;
            double joint0_angle = joint_angles(0);
            double joint1_angle = joint_angles(1);
            double joint2_angle = joint_angles(2);
            double joint3_angle = joint_angles(3);
            double joint4_angle = joint_angles(4) + joint3_angle;

            if (leg == 0)
            {
                side = 1.0;
            }
            else
            {
                side = -1.0;
            }

            J(0, 0) = 0.0005 * sin(joint0_angle) * (440.0 * sin(joint2_angle + joint3_angle) + 80.0 * sin(joint2_angle + joint4_angle) + 440.0 * sin(joint2_angle) + 27.0) - 1.0 * cos(joint0_angle) * (0.015 * side + 0.22 * cos(joint2_angle) * sin(joint1_angle) + 0.0205 * side * cos(joint1_angle) - 0.22 * sin(joint1_angle) * sin(joint2_angle) * sin(joint3_angle) - 0.04 * sin(joint1_angle) * sin(joint2_angle) * sin(joint4_angle) + 0.22 * cos(joint2_angle) * cos(joint3_angle) * sin(joint1_angle) + 0.04 * cos(joint2_angle) * cos(joint4_angle) * sin(joint1_angle));
            J(1, 0) = -0.0005 * cos(joint0_angle) * (440.0 * sin(joint2_angle + joint3_angle) + 80.0 * sin(joint2_angle + joint4_angle) + 440.0 * sin(joint2_angle) + 27.0) - 1.0 * sin(joint0_angle) * (0.015 * side + 0.22 * cos(joint2_angle) * sin(joint1_angle) + 0.0205 * side * cos(joint1_angle) - 0.22 * sin(joint1_angle) * sin(joint2_angle) * sin(joint3_angle) - 0.04 * sin(joint1_angle) * sin(joint2_angle) * sin(joint4_angle) + 0.22 * cos(joint2_angle) * cos(joint3_angle) * sin(joint1_angle) + 0.04 * cos(joint2_angle) * cos(joint4_angle) * sin(joint1_angle));
            J(2, 0) = 0.0;
            J(3, 0) = 0.0;
            J(4, 0) = 0.0;
            J(5, 0) = 1.0;

            J(0, 1) = 1.0 * sin(joint0_angle) * (cos(joint1_angle) * (1.0 * sin(joint2_angle) * (sin(joint3_angle) * (0.04 * cos(joint3_angle - 1.0 * joint4_angle) + 0.22) - 0.04 * sin(joint3_angle - 1.0 * joint4_angle) * cos(joint3_angle)) - cos(joint2_angle) * (0.04 * sin(joint3_angle - 1.0 * joint4_angle) * sin(joint3_angle) + cos(joint3_angle) * (0.04 * cos(joint3_angle - 1.0 * joint4_angle) + 0.22) + 0.22)) + 0.0205 * side * sin(joint1_angle));
            J(1, 1) = -cos(joint0_angle) * (cos(joint1_angle) * (1.0 * sin(joint2_angle) * (sin(joint3_angle) * (0.04 * cos(joint3_angle - 1.0 * joint4_angle) + 0.22) - 0.04 * sin(joint3_angle - 1.0 * joint4_angle) * cos(joint3_angle)) - cos(joint2_angle) * (0.04 * sin(joint3_angle - 1.0 * joint4_angle) * sin(joint3_angle) + cos(joint3_angle) * (0.04 * cos(joint3_angle - 1.0 * joint4_angle) + 0.22) + 0.22)) + 0.0205 * side * sin(joint1_angle));
            J(2, 1) = 0.22 * cos(joint2_angle) * sin(joint1_angle) + 0.0205 * side * cos(joint1_angle) - 0.22 * sin(joint1_angle) * sin(joint2_angle) * sin(joint3_angle) - 0.04 * sin(joint1_angle) * sin(joint2_angle) * sin(joint4_angle) + 0.22 * cos(joint2_angle) * cos(joint3_angle) * sin(joint1_angle) + 0.04 * cos(joint2_angle) * cos(joint4_angle) * sin(joint1_angle);
            J(3, 1) = cos(joint0_angle);
            J(4, 1) = sin(joint0_angle);
            J(5, 1) = 0.0;

            J(0, 2) = sin(joint0_angle) * sin(joint1_angle) * (0.22 * sin(joint2_angle + joint3_angle) + 0.04 * sin(joint2_angle + joint4_angle) + 0.22 * sin(joint2_angle)) - 0.02 * cos(joint0_angle) * (11.0 * cos(joint2_angle + joint3_angle) + 2.0 * cos(joint2_angle + joint4_angle) + 11.0 * cos(joint2_angle));
            J(1, 2) = -0.02 * sin(joint0_angle) * (11.0 * cos(joint2_angle + joint3_angle) + 2.0 * cos(joint2_angle + joint4_angle) + 11.0 * cos(joint2_angle)) - 1.0 * cos(joint0_angle) * sin(joint1_angle) * (0.22 * sin(joint2_angle + joint3_angle) + 0.04 * sin(joint2_angle + joint4_angle) + 0.22 * sin(joint2_angle));
            J(2, 2) = 0.02 * cos(joint1_angle) * (11.0 * sin(joint2_angle + joint3_angle) + 2.0 * sin(joint2_angle + joint4_angle) + 11.0 * sin(joint2_angle));
            J(3, 2) = -1.0 * cos(joint1_angle) * sin(joint0_angle);
            J(4, 2) = cos(joint0_angle) * cos(joint1_angle);
            J(5, 2) = sin(joint1_angle);

            J(0, 3) = 0.22 * cos(joint0_angle) * sin(joint2_angle) * sin(joint3_angle) - 0.22 * cos(joint0_angle) * cos(joint2_angle) * cos(joint3_angle) + 0.22 * cos(joint2_angle) * sin(joint0_angle) * sin(joint1_angle) * sin(joint3_angle) + 0.22 * cos(joint3_angle) * sin(joint0_angle) * sin(joint1_angle) * sin(joint2_angle);
            J(1, 3) = 0.22 * sin(joint0_angle) * sin(joint2_angle) * sin(joint3_angle) - 0.22 * cos(joint2_angle) * cos(joint3_angle) * sin(joint0_angle) - 0.22 * cos(joint0_angle) * cos(joint2_angle) * sin(joint1_angle) * sin(joint3_angle) - 0.22 * cos(joint0_angle) * cos(joint3_angle) * sin(joint1_angle) * sin(joint2_angle);
            J(2, 3) = 0.22 * sin(joint2_angle + joint3_angle) * cos(joint1_angle);
            J(3, 3) = -1.0 * cos(joint1_angle) * sin(joint0_angle);
            J(4, 3) = cos(joint0_angle) * cos(joint1_angle);
            J(5, 3) = sin(joint1_angle);

            J(0, 4) = 0.04 * cos(joint0_angle) * sin(joint2_angle) * sin(joint4_angle) - 0.04 * cos(joint0_angle) * cos(joint2_angle) * cos(joint4_angle) + 0.04 * cos(joint2_angle) * sin(joint0_angle) * sin(joint1_angle) * sin(joint4_angle) + 0.04 * cos(joint4_angle) * sin(joint0_angle) * sin(joint1_angle) * sin(joint2_angle);
            J(1, 4) = 0.04 * sin(joint0_angle) * sin(joint2_angle) * sin(joint4_angle) - 0.04 * cos(joint2_angle) * cos(joint4_angle) * sin(joint0_angle) - 0.04 * cos(joint0_angle) * cos(joint2_angle) * sin(joint1_angle) * sin(joint4_angle) - 0.04 * cos(joint0_angle) * cos(joint4_angle) * sin(joint1_angle) * sin(joint2_angle);
            J(2, 4) = 0.04 * sin(joint2_angle + joint4_angle) * cos(joint1_angle);
            J(3, 4) = -1.0 * cos(joint1_angle) * sin(joint0_angle);
            J(4, 4) = cos(joint0_angle) * cos(joint1_angle);
            J(5, 4) = sin(joint1_angle);

        }

        return J;

    }




    Vec3<double> ForwardKinematics(Vec5<double> &joint_angles, int leg)
    {

        Vec3<double> CoM2Foot;
        if (leg == 0) // left leg
        {
            CoM2Foot = getHip2Location(0) + HiptoFoot(joint_angles, 0);
        }
        else
        { // right leg
            CoM2Foot = getHip2Location(1) + HiptoFoot(joint_angles, 1);
        }
        return CoM2Foot;
    }




    // Vec5<double> InverseKinematics(Vec3<double> &p_Hip2Foot, int leg)
    // {
    //     Vec5<double> q; //joint angles of hip roll, hip pitch, knee pitch. hip yaw and ankle pitch are assumed to be 0.

    //     double l1 = hipLinkLength; // ab_ad
    //     double l2 = thighLinkLength;
    //     double l3 = calfLinkLength;

    //     double x = p_Hip2Foot[0];
    //     double y = p_Hip2Foot[1];
    //     double z = p_Hip2Foot[2];

    //     // std::cout << "Testing test" << std::endl;

    //     double z_1 = sqrt(y * y + z * z - l1 * l1);
    //     double theta_r = atan2(x, z_1);

    //     // std::cout << "theta: " << theta_r << std::endl;
    //     // std::cout << z_1 << std::endl;
    //     double q0 = 0;
    //     double q1 = 2 * atan((l1 - sqrt(-z * z + z_1 * z_1 + l1 * l1)) / (abs(z) + z_1));
    //     double q2 = acos((l2 * l2 + x * x + z_1 * z_1 - l3 * l3) / (2 * l2 * sqrt(z_1 * z_1 + x * x))) + theta_r;
    //     double q3 = acos((x * x + y * y + z * z - l1 * l1 - l2 * l2 - l3 * l3) / (2 * l2 * l3));
    //     double q4 = -q1 - q2;

    //     if (leg == 0)
    //     {
    //         q(0) = -q0;
    //         q(1) = q1;
    //         q(2) = q2;
    //         q(3) = q3;
    //         q(4) = q4;
    //     }

    //     if (leg == 1)
    //     {
    //         q(0) = q0;
    //         q(1) = q1;
    //         q(2) = q2;
    //         q(3) = q3;
    //         q(4) = q4;
    //     }


    //     return q;
    // }


    // //It is a customized inverse kinematics function provided from USC specifically designed for swing leg control
    // // TODO: Verify which one between the above and this one is correct
    Vec3<double> InverseKinematics_swingctrl(Vec3<double> &p_Hip2Foot, int leg)
    {
        Vec3<double> q; //joint angles of hip roll, hip pitch, knee pitch. hip yaw and ankle pitch are assumed to be 0.

        double side; 
        if (leg == 0) {
            side = 1.0;
        }
        else if (leg == 1) {
            side = -1.0;
        }

        Vec3<double> hip_roll;
            hip_roll = {0.0465, 0.02*side, -0.197};
        Vec3<double> foot_des_to_hip_roll = p_Hip2Foot - hip_roll;
        double distance_3D = pow( (  pow((foot_des_to_hip_roll(0)+0.06),2.0) + 
                    pow(foot_des_to_hip_roll(1), 2.0) + pow(foot_des_to_hip_roll(2), 2.0) ), 0.5);
        double distance_2D_yOz = pow( ( pow(foot_des_to_hip_roll(1), 2.0) + pow(foot_des_to_hip_roll(2),2.0) ), 0.5 );
        double distance_horizontal = 0.0205;
        double distance_vertical = pow(( pow(distance_2D_yOz,2.0)-pow(distance_horizontal,2.0)), 0.5);
        double distance_2D_xOz = pow(( pow(distance_3D,2.0)-pow(distance_horizontal,2.0)), 0.5);
        
        // qDes
        q(0) = std::asin( foot_des_to_hip_roll(1) / distance_2D_yOz ) + std::asin( distance_horizontal*side / distance_2D_yOz );        
        q(1) = std::acos(distance_2D_xOz / 2.0 / 0.22) - std::acos(distance_vertical / distance_2D_xOz) * (foot_des_to_hip_roll(0)+0.06) / abs((foot_des_to_hip_roll(0)+0.06));
        q(2) = 2.0 * std::asin(distance_2D_xOz / 2.0 / 0.22) - 3.14159;

        return q;

    }

    // adapted from simulation codebase
    Vec3<double> ComputeIK(Vec3<double> &p_Hip2Foot, int leg)
    {
        Vec3<double> q; //joint angles of hip roll, hip pitch, knee pitch. hip yaw and ankle pitch are assumed to be 0.

        double side; 
        if (leg == 0) {
            side = -1.0;
        }
        else if (leg == 1) {
            side = 1.0;
        }
        Vec3<double> L_hipRollLocation = {0.0465, 0.015, -0.0705};
        Vec3<double> L_hipYawLocation = {-0.005, -0.047, -0.126};
        Eigen::Vector3d hip_roll(L_hipRollLocation[0]-0.06, 0.0, L_hipYawLocation[2]+L_hipRollLocation[2]*2);
        Eigen::Vector3d foot_des_to_hip_roll = p_Hip2Foot - hip_roll; //in hip roll frame
        double distance_3D = foot_des_to_hip_roll.norm();
        double distance_2D_yOz = std::sqrt(std::pow(foot_des_to_hip_roll[1], 2) + std::pow(foot_des_to_hip_roll[2], 2));
        double distance_horizontal = 0.0205;
        double distance_vertical = std::sqrt(std::max(0.00001, std::pow(distance_2D_yOz, 2) - std::pow(distance_horizontal, 2)));        // double distance_vertical = std::sqrt(std::pow(distance_2D_yOz, 2) - std::pow(distance_horizontal, 2));
        double distance_2D_xOz = pow(( pow(distance_3D,2.0)-pow(distance_horizontal,2.0)), 0.5);
                       
        // Ensure arguments are within valid range for acos and asin
        double acosArg1 = clamp(distance_2D_xOz / (2.0 * 0.22), -1.0, 1.0);
        double acosArg2 = clamp(distance_vertical / distance_2D_xOz, -1.0, 1.0);
        double divisor = std::abs(foot_des_to_hip_roll[0]);
        divisor = (divisor == 0.0) ? 1e-6 : divisor; // Prevent division by zero
        
        // qDes
        q(0) = std::asin(clamp(foot_des_to_hip_roll(1) / distance_2D_yOz, -1.0, 1.0)) + std::asin(clamp(distance_horizontal*side / distance_2D_yOz, -1.0, 1.0));
        q(1) = std::acos(acosArg1) - std::acos(acosArg2) * (foot_des_to_hip_roll[0]) / divisor;
        q(2) = 2.0 * std::asin(clamp(distance_2D_xOz / 2.0 / 0.22, -1.0, 1.0)) - 3.14159;

        return q;

    }

    double clamp(double val, double minVal, double maxVal) {
                return std::max(minVal, std::min(val, maxVal));
        }

};

#endif
