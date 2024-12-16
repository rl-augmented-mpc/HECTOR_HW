#include "../../include/common/LegController.h"

#include <iostream>
#include <chrono>

// upper level of joint controller 
// send data to joint controller

void LegControllerCommand::zero(){
    tau = Vec5<double>::Zero();
    qDes = Vec5<double>::Zero();
    qdDes = Vec5<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat5<double>::Zero();
    kdJoint = Mat5<double>::Zero();
    double kptoe = 0;
    double kdtoe = 0;
}

/*!
 * Zero leg data
 */ 
void LegControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J = Mat65<double>::Zero();
    J2 = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

void LegController::zeroCommand(){
    for (int i = 0; i<2; i++){
        commands[i].zero();
    }
}

void LegController::updateData(const LowlevelState* state, double* offset){

    // LIDAR-modified
    // Joint sequence reassignment: low-lev joint data -> controller space joint data
    for (int leg = 0; leg < 2; leg++)
    {

        // get from low-lev motorstate
        for (int j = 0; j < 5; j++)
        {
            data[leg].q(j) = state->motorState[motor_sequence[j + leg * 5]].q;
            data[leg].qd(j) = state->motorState[motor_sequence[j + leg * 5]].dq;
            data[leg].tau(j) = state->motorState[motor_sequence[j + leg * 5]].tauEst;

            if (state->motorState[motor_sequence[j + leg * 5]].q == 0)
            {
                std::cout << "\nMotor Connection on leg " << leg << " at " << j << " LOST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                // abort();
            }
        }

        std::cout << "\n\nleg " << leg << " angle raw data: " << std::endl;
        for (int j = 0; j < 5; j++)
        {
            std::cout << data[leg].q(j) * 180 / 3.1415 << "    ";
        }

        // offset importing
        for (int i = 0; i < 5; i++)
        {
            data[leg].q(i) = data[leg].q(i) + offset[i + leg * 5];
        }

        // Unitree-related calibration
        if (leg == 1)
        {
            data[leg].q(0) = -data[leg].q(0);
            data[leg].q(1) = -data[leg].q(1);
            data[leg].q(2) = -data[leg].q(2);
            data[leg].qd(0) = -data[leg].qd(0);
            data[leg].qd(1) = -data[leg].qd(1);
            data[leg].qd(2) = -data[leg].qd(2);
        }

        // For knee pully system
        double compensation = 1.0;
        data[leg].q(3) = data[leg].q(3) / gear_ratio;
        data[leg].qd(3) = data[leg].qd(3) / gear_ratio;
        data[leg].tau(3) = -data[leg].tau(3) * gear_ratio * compensation;

        // For Ankle actuation linkage
        data[leg].q(4) = data[leg].q(4) - data[leg].q(3);
        data[leg].qd(4) = data[leg].qd(4) - data[leg].qd(3);

        computeLegJacobianAndPosition(_biped, data[leg].q, &(data[leg].J), &(data[leg].J2), &(data[leg].p), leg);
        // data[leg].J = _robot.HiptoFootJacobian(data[leg].q, leg, 1);
        // data[leg].J2 = data[leg].J.block(0,0, 3,5);
        // data[leg].p = _robot.HiptoFoot(data[leg].q, leg, 1);
        data[leg].v = data[leg].J2 * data[leg].qd;

        for (int i = 0; i < 5; i++)
        {
            feedback_torque << data[leg].tau(i) << "    ";
        }

        std::cout << "\nleg " << leg << " angle calibrated data: " << std::endl;
        for (int j = 0; j < 5; j++)
        {
            std::cout << data[leg].q(j) * 180 / 3.1415 << "    ";
        }
    }
    feedback_torque << std::endl;
}








void LegController::updateCommand(LowlevelCmd* cmd, double* offset, int motiontime){
    
    

    //For controller-configuired q, qd, tau, all the axis of rotations are in the same direction; +z, +x, +y.

    auto start = std::chrono::steady_clock::now();

    bool PDStand = 0;

    //LIDAR-modified
    //Necessary: To prevent actuating unused joints 
    for (int i = 0; i < 12; i++){
        cmd->motorCmd[i].tau = 0;
    }

    for (int leg = 0; leg < 2; leg++){


        // Stance foot force to torque mapping======================================================================
        Vec6<double> footWrench =  commands[leg].feedforwardForce;
        Vec5<double> legTorque = (data[leg].J.transpose() * footWrench);
        commands[leg].tau = legTorque;


        for (int i = 0; i < 6; i++){
            feedforward_force << commands[leg].feedforwardForce(i) << " ";
        }
        // Torque ramping up for first few seconds==============================================================
        double percent;
        double duration = 4000;
        percent = (double)(motiontime-1000)/duration;
        if (motiontime < 1000) {
            percent = 0.0;
        }
        if (motiontime > 1000+duration) {
            percent = 1.0;
        }

        commands[leg].tau(0) = legTorque[0] * percent;
        commands[leg].tau(1) = legTorque[1] * percent;
        commands[leg].tau(2) = legTorque[2] * percent;
        commands[leg].tau(3) = legTorque[3] * percent;
        commands[leg].tau(4) = legTorque[4] * percent;

        



        // Decides joint PD control======================================================================================

        if ( commands[leg].pDes(2)==0 ){ //means contact
            JointPDSwitch = 0.0;

        }else if( commands[leg].vDes(2) == 1 ){ //for PDStand

            PDStand = 1;
            JointPDSwitch = 1.0;

            
        
        }else{
            JointPDSwitch = 1.0;
        }

        
        // //// Joint PD gains //
        double Kp_joint[5] = {10.0, 30.0, 30.0, 30.0, 10.};
                           // = {80.0, 80.0, 100.0, 120.0, 20.0};
        double Kd_joint[5] //= {1, 1.0, 1.5, 1.5, 0.5};
                           = {1.0, 1.0, 1.0, 1.0, 0.5};
                           //= {4, 4, 6, 6, 2};
        //might need to be increased


        double PDStand_percent = 1;
        // Swing foot===========================================================================================
        if (!PDStand){

            //Doing IK to get hip roll, pitch, knee
            //with the given desired foot pos/vel and fixing the hip yaw & ankle pitch (to make it degenerated to 3 DOFs)

            Vec3<double> foot_des = commands[leg].pDes; 
            Vec3<double> foot_v_des = commands[leg].vDes;

            //qDes
            commands[leg].qDes.block(1,0, 3,1) = InverseKinematics_swingctrl(foot_des, leg);
            // hip yaw = 0, ankle pitch is designed to be flat(=0)
            commands[leg].qDes(0) = 0;
            commands[leg].qDes(4) = -data[leg].q(3) - data[leg].q(2);

            // qdDes
            commands[leg].qdDes = data[leg].J2.transpose() * foot_v_des;

        }
        else
        {
            for (int k = 0; k < 5; k ++) {
                Kp_joint[k] = Kp_joint[k] * 1;
                Kd_joint[k] = Kd_joint[k] * 1;
                //  std::cout << "PD Standing in legctrl!!!!!!!!!!!!" << commands[leg].qDes(k) << std::endl;
                 
            }
            JointPDSwitch = 1.0;
        }











        // Commands to low-level controller======================================================================
        // Assigning to lowCmd first and then does the re-mapping to avoid corrupting controller-configured commands
        for (int k = 0; k < 5; k ++) {
            cmd->motorCmd[motor_sequence[k + leg*5]].tau = commands[leg].tau(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].q = commands[leg].qDes(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].dq = commands[leg].qdDes(k) * JointPDSwitch;
            cmd->motorCmd[motor_sequence[k + leg*5]].Kp = Kp_joint[k] * JointPDSwitch * percent;
            cmd->motorCmd[motor_sequence[k + leg*5]].Kd = Kd_joint[k] * 1.25;
        }

        //Command re-mapping: the reversed order of reading data===============================================

        cmd->motorCmd[motor_sequence[4 + leg*5]].q   += cmd->motorCmd[motor_sequence[3 + leg*5]].q ;
        cmd->motorCmd[motor_sequence[4 + leg*5]].dq  += cmd->motorCmd[motor_sequence[3 + leg*5]].dq;

        cmd->motorCmd[motor_sequence[3 + leg*5]].q   *= gear_ratio;
        cmd->motorCmd[motor_sequence[3 + leg*5]].dq  *= gear_ratio;
        cmd->motorCmd[motor_sequence[3 + leg*5]].tau /= gear_ratio;
        cmd->motorCmd[motor_sequence[3 + leg*5]].tau *= beltCompRatio;

        // Unitree-related
        if (leg == 1) {
            cmd->motorCmd[motor_sequence[0 + leg*5]].q   *= -1.0;
            cmd->motorCmd[motor_sequence[1 + leg*5]].q   *= -1.0;
            cmd->motorCmd[motor_sequence[2 + leg*5]].q   *= -1.0;

            cmd->motorCmd[motor_sequence[0 + leg*5]].dq   *= -1.0;
            cmd->motorCmd[motor_sequence[1 + leg*5]].dq   *= -1.0;
            cmd->motorCmd[motor_sequence[2 + leg*5]].dq   *= -1.0;

            cmd->motorCmd[motor_sequence[0 + leg*5]].tau   *= -1.0;
            cmd->motorCmd[motor_sequence[1 + leg*5]].tau   *= -1.0;
            cmd->motorCmd[motor_sequence[2 + leg*5]].tau   *= -1.0;
        }

        //offset 
        for(int i = 0; i < 5; i++){
            cmd->motorCmd[motor_sequence[i + leg*5]].q -= offset[i+leg*5];
        }

        //For unused actuators - could be removed after testing
        cmd->motorCmd[0].tau = 0;
        cmd->motorCmd[3].tau = 0;
        cmd->motorCmd[0].Kp = 0;
        cmd->motorCmd[0].Kd = 5;
        cmd->motorCmd[3].Kp = 0;
        cmd->motorCmd[3].Kd = 5;

    }


    auto end =std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Execution time for legController(): " << duration.count() << " microseconds" << std::endl;
    tau_leg_controller << std::endl;
    feedforward_force << std::endl;


    
    



   
}

void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J, Mat35<double>* J2, Vec3<double>* p, int leg)
{

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    // double q4 = q(4);
    // for coupled Jacobian:
    double q4 = q(4)+q3;

    double side; // 1 for Left legs; -1 for right legs
    if (leg == 1){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;}
    else if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = -1.0;
    }
    // std::cout<< "Leg Sign" << side << std::endl;
    

    if(J){
    // J->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    // J->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    // J->operator()(2, 0) =  0.0;
    // J->operator()(3, 0) = 0.0;
    // J->operator()(4, 0) = 0.0;
    // J->operator()(5, 0) = 1.0;

    // J->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    // J->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    // J->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);
    // J->operator()(3, 1) = cos(q0);
    // J->operator()(4, 1) = sin(q0);
    // J->operator()(5, 1) = 0.0;

    // J->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    // J->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    // J->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    // J->operator()(3, 2) = -cos(q1)*sin(q0);
    // J->operator()(4, 2) = cos(q0)*cos(q1);
    // J->operator()(5, 2) = sin(q1);

    // J->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    // J->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    // J->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    // J->operator()(3, 3) = -cos(q1)*sin(q0);
    // J->operator()(4, 3) = cos(q0)*cos(q1);
    // J->operator()(5, 3) = sin(q1);

    // J->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    // J->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    // J->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    // J->operator()(3, 4) = -cos(q1)*sin(q0);
    // J->operator()(4, 4) = cos(q0)*cos(q1);
    // J->operator()(5, 4) = sin(q1);

    // consider knee-ankle coupling:
    J->operator()(0,0) = 0.0005*sin(q0)*(440.0*sin(q2 + q3) + 80.0*sin(q2 + q4) + 440.0*sin(q2) + 27.0) - 1.0*cos(q0)*(0.015*side + 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1));
    J->operator()(1,0) = - 0.0005*cos(q0)*(440.0*sin(q2 + q3) + 80.0*sin(q2 + q4) + 440.0*sin(q2) + 27.0) - 1.0*sin(q0)*(0.015*side + 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1));
    J->operator()(2,0) = 0.0;
    J->operator()(3,0) = 0.0;
    J->operator()(4,0) = 0.0;
    J->operator()(5,0) = 1.0;
    
    J->operator()(0,1) = 1.0*sin(q0)*(cos(q1)*(1.0*sin(q2)*(sin(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) - 0.04*sin(q3 - 1.0*q4)*cos(q3)) - cos(q2)*(0.04*sin(q3 - 1.0*q4)*sin(q3) + cos(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) + 0.22)) + 0.0205*side*sin(q1));
    J->operator()(1,1) = -cos(q0)*(cos(q1)*(1.0*sin(q2)*(sin(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) - 0.04*sin(q3 - 1.0*q4)*cos(q3)) - cos(q2)*(0.04*sin(q3 - 1.0*q4)*sin(q3) + cos(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) + 0.22)) + 0.0205*side*sin(q1));
    J->operator()(2,1) = 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1);
    J->operator()(3,1) = cos(q0);
    J->operator()(4,1) = sin(q0);
    J->operator()(5,1) = 0.0;
    
    J->operator()(0,2) = sin(q0)*sin(q1)*(0.22*sin(q2 + q3) + 0.04*sin(q2 + q4) + 0.22*sin(q2)) - 0.02*cos(q0)*(11.0*cos(q2 + q3) + 2.0*cos(q2 + q4) + 11.0*cos(q2));
    J->operator()(1,2) = - 0.02*sin(q0)*(11.0*cos(q2 + q3) + 2.0*cos(q2 + q4) + 11.0*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.22*sin(q2 + q3) + 0.04*sin(q2 + q4) + 0.22*sin(q2));
    J->operator()(2,2) = 0.02*cos(q1)*(11.0*sin(q2 + q3) + 2.0*sin(q2 + q4) + 11.0*sin(q2));
    J->operator()(3,2) = -1.0*cos(q1)*sin(q0);
    J->operator()(4,2) = cos(q0)*cos(q1);
    J->operator()(5,2) = sin(q1);
    
    J->operator()(0,3) = 0.22*cos(q0)*sin(q2)*sin(q3) - 0.22*cos(q0)*cos(q2)*cos(q3) + 0.22*cos(q2)*sin(q0)*sin(q1)*sin(q3) + 0.22*cos(q3)*sin(q0)*sin(q1)*sin(q2);
    J->operator()(1,3) = 0.22*sin(q0)*sin(q2)*sin(q3) - 0.22*cos(q2)*cos(q3)*sin(q0) - 0.22*cos(q0)*cos(q2)*sin(q1)*sin(q3) - 0.22*cos(q0)*cos(q3)*sin(q1)*sin(q2);
    J->operator()(2,3) = 0.22*sin(q2 + q3)*cos(q1);
    J->operator()(3,3) = -1.0*cos(q1)*sin(q0);
    J->operator()(4,3) = cos(q0)*cos(q1);
    J->operator()(5,3) = sin(q1);
    
    J->operator()(0,4) = 0.04*cos(q0)*sin(q2)*sin(q4) - 0.04*cos(q0)*cos(q2)*cos(q4) + 0.04*cos(q2)*sin(q0)*sin(q1)*sin(q4) + 0.04*cos(q4)*sin(q0)*sin(q1)*sin(q2);
    J->operator()(1,4) = 0.04*sin(q0)*sin(q2)*sin(q4) - 0.04*cos(q2)*cos(q4)*sin(q0) - 0.04*cos(q0)*cos(q2)*sin(q1)*sin(q4) - 0.04*cos(q0)*cos(q4)*sin(q1)*sin(q2);
    J->operator()(2,4) = 0.04*sin(q2 + q4)*cos(q1);
    J->operator()(3,4) = -1.0*cos(q1)*sin(q0);
    J->operator()(4,4) = cos(q0)*cos(q1);
    J->operator()(5,4) = sin(q1);
   }
   
//    side *= -1;

   if(J2){
    // J->operator()(0,0) = 0.0005*sin(q0)*(440.0*sin(q2 + q3) + 80.0*sin(q2 + q4) + 440.0*sin(q2) + 27.0) - 1.0*cos(q0)*(0.015*side + 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1));
    // J->operator()(1,0) = - 0.0005*cos(q0)*(440.0*sin(q2 + q3) + 80.0*sin(q2 + q4) + 440.0*sin(q2) + 27.0) - 1.0*sin(q0)*(0.015*side + 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1));
    // J->operator()(2,0) = 0.0;
    
    // J->operator()(0,1) = 1.0*sin(q0)*(cos(q1)*(1.0*sin(q2)*(sin(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) - 0.04*sin(q3 - 1.0*q4)*cos(q3)) - cos(q2)*(0.04*sin(q3 - 1.0*q4)*sin(q3) + cos(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) + 0.22)) + 0.0205*side*sin(q1));
    // J->operator()(1,1) = -cos(q0)*(cos(q1)*(1.0*sin(q2)*(sin(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) - 0.04*sin(q3 - 1.0*q4)*cos(q3)) - cos(q2)*(0.04*sin(q3 - 1.0*q4)*sin(q3) + cos(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) + 0.22)) + 0.0205*side*sin(q1));
    // J->operator()(2,1) = 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1);
    
    // J->operator()(0,2) = sin(q0)*sin(q1)*(0.22*sin(q2 + q3) + 0.04*sin(q2 + q4) + 0.22*sin(q2)) - 0.02*cos(q0)*(11.0*cos(q2 + q3) + 2.0*cos(q2 + q4) + 11.0*cos(q2));
    // J->operator()(1,2) = - 0.02*sin(q0)*(11.0*cos(q2 + q3) + 2.0*cos(q2 + q4) + 11.0*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.22*sin(q2 + q3) + 0.04*sin(q2 + q4) + 0.22*sin(q2));
    // J->operator()(2,2) = 0.02*cos(q1)*(11.0*sin(q2 + q3) + 2.0*sin(q2 + q4) + 11.0*sin(q2));
    
    // J->operator()(0,3) = 0.22*cos(q0)*sin(q2)*sin(q3) - 0.22*cos(q0)*cos(q2)*cos(q3) + 0.22*cos(q2)*sin(q0)*sin(q1)*sin(q3) + 0.22*cos(q3)*sin(q0)*sin(q1)*sin(q2);
    // J->operator()(1,3) = 0.22*sin(q0)*sin(q2)*sin(q3) - 0.22*cos(q2)*cos(q3)*sin(q0) - 0.22*cos(q0)*cos(q2)*sin(q1)*sin(q3) - 0.22*cos(q0)*cos(q3)*sin(q1)*sin(q2);
    // J->operator()(2,3) = 0.22*sin(q2 + q3)*cos(q1);
    
    // J->operator()(0,4) = 0.04*cos(q0)*sin(q2)*sin(q4) - 0.04*cos(q0)*cos(q2)*cos(q4) + 0.04*cos(q2)*sin(q0)*sin(q1)*sin(q4) + 0.04*cos(q4)*sin(q0)*sin(q1)*sin(q2);
    // J->operator()(1,4) = 0.04*sin(q0)*sin(q2)*sin(q4) - 0.04*cos(q2)*cos(q4)*sin(q0) - 0.04*cos(q0)*cos(q2)*sin(q1)*sin(q4) - 0.04*cos(q0)*cos(q4)*sin(q1)*sin(q2);
    // J->operator()(2,4) = 0.04*sin(q2 + q4)*cos(q1);
 

    // consider knee-ankle coupling:
    J->operator()(0,0) = 0.0005*sin(q0)*(440.0*sin(q2 + q3) + 80.0*sin(q2 + q4) + 440.0*sin(q2) + 27.0) - 1.0*cos(q0)*(0.015*side + 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1));
    J->operator()(1,0) = - 0.0005*cos(q0)*(440.0*sin(q2 + q3) + 80.0*sin(q2 + q4) + 440.0*sin(q2) + 27.0) - 1.0*sin(q0)*(0.015*side + 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1));
    J->operator()(2,0) = 0.0;
    
    J->operator()(0,1) = 1.0*sin(q0)*(cos(q1)*(1.0*sin(q2)*(sin(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) - 0.04*sin(q3 - 1.0*q4)*cos(q3)) - cos(q2)*(0.04*sin(q3 - 1.0*q4)*sin(q3) + cos(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) + 0.22)) + 0.0205*side*sin(q1));
    J->operator()(1,1) = -cos(q0)*(cos(q1)*(1.0*sin(q2)*(sin(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) - 0.04*sin(q3 - 1.0*q4)*cos(q3)) - cos(q2)*(0.04*sin(q3 - 1.0*q4)*sin(q3) + cos(q3)*(0.04*cos(q3 - 1.0*q4) + 0.22) + 0.22)) + 0.0205*side*sin(q1));
    J->operator()(2,1) = 0.22*cos(q2)*sin(q1) + 0.0205*side*cos(q1) - 0.22*sin(q1)*sin(q2)*sin(q3) - 0.04*sin(q1)*sin(q2)*sin(q4) + 0.22*cos(q2)*cos(q3)*sin(q1) + 0.04*cos(q2)*cos(q4)*sin(q1);
    
    J->operator()(0,2) = sin(q0)*sin(q1)*(0.22*sin(q2 + q3) + 0.04*sin(q2 + q4) + 0.22*sin(q2)) - 0.02*cos(q0)*(11.0*cos(q2 + q3) + 2.0*cos(q2 + q4) + 11.0*cos(q2));
    J->operator()(1,2) = - 0.02*sin(q0)*(11.0*cos(q2 + q3) + 2.0*cos(q2 + q4) + 11.0*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.22*sin(q2 + q3) + 0.04*sin(q2 + q4) + 0.22*sin(q2));
    J->operator()(2,2) = 0.02*cos(q1)*(11.0*sin(q2 + q3) + 2.0*sin(q2 + q4) + 11.0*sin(q2));
    
    J->operator()(0,3) = 0.22*cos(q0)*sin(q2)*sin(q3) - 0.22*cos(q0)*cos(q2)*cos(q3) + 0.22*cos(q2)*sin(q0)*sin(q1)*sin(q3) + 0.22*cos(q3)*sin(q0)*sin(q1)*sin(q2);
    J->operator()(1,3) = 0.22*sin(q0)*sin(q2)*sin(q3) - 0.22*cos(q2)*cos(q3)*sin(q0) - 0.22*cos(q0)*cos(q2)*sin(q1)*sin(q3) - 0.22*cos(q0)*cos(q3)*sin(q1)*sin(q2);
    J->operator()(2,3) = 0.22*sin(q2 + q3)*cos(q1);
    
    J->operator()(0,4) = 0.04*cos(q0)*sin(q2)*sin(q4) - 0.04*cos(q0)*cos(q2)*cos(q4) + 0.04*cos(q2)*sin(q0)*sin(q1)*sin(q4) + 0.04*cos(q4)*sin(q0)*sin(q1)*sin(q2);
    J->operator()(1,4) = 0.04*sin(q0)*sin(q2)*sin(q4) - 0.04*cos(q2)*cos(q4)*sin(q0) - 0.04*cos(q0)*cos(q2)*sin(q1)*sin(q4) - 0.04*cos(q0)*cos(q4)*sin(q1)*sin(q2);
    J->operator()(2,4) = 0.04*sin(q2 + q4)*cos(q1);
 
    }


   if(p){
    p->operator()(0) = - (3*cos(q0))/200 - (9*sin(q4)*(cos(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))))/250 - (11*cos(q0)*sin(q2))/50 - ( (side)*sin(q0))/50 - (11*cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))/50 - (11*sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2))))/250 - (23*cos(q1)* (side)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50;
    p->operator()(1) = (cos(q0)* (side))/50 - (9*sin(q4)*(cos(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1))))/250 - (3*sin(q0))/200 - (11*sin(q0)*sin(q2))/50 - (11*cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)))/50 - (11*sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2))))/250 + (23*cos(q0)*cos(q1)* (side))/1000 + (11*cos(q0)*cos(q2)*sin(q1))/50;
    p->operator()(2) = (23*(side)*sin(q1))/1000 - (11*cos(q1)*cos(q2))/50 - (9*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/250 + (9*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/250 - (11*cos(q1)*cos(q2)*cos(q3))/50 + (11*cos(q1)*sin(q2)*sin(q3))/50 - 3.0/50.0;
   }   

}





Vec3<double> InverseKinematics_swingctrl(Vec3<double> &p_Hip2Foot, int leg)
{
    Vec3<double> q; //joint angles of hip roll, hip pitch, knee pitch. hip yaw and ankle pitch are assumed to be 0.

    double side; 
    if (leg == 0) {
        side = -1.0;
    }
    else if (leg == 1) {
        side = 1.0;
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