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
    kptoe = 0;
    kdtoe = 0;

    which_control = 0;
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

void LegController::updateData(const LowlevelState* state){

    // LIDAR-modified
    // Joint sequence reassignment: low-lev joint data -> controller space joint data
    int *motor_sequence = _biped.motor_sequence;

    for (int leg = 0; leg < 2; leg++)
    {
        // get from low-lev motorstate
        for (int j = 0; j < 5; j++)
        {
            data[leg].q(j) = state->motorState[motor_sequence[j + leg * 5]].q;
            data[leg].qd(j) = state->motorState[motor_sequence[j + leg * 5]].dq;
            data[leg].tau(j) = state->motorState[motor_sequence[j + leg * 5]].tauEst;

        }

        if (!_biped.CheckMotorConnection(leg, data[leg].q)){
            // abort();
        }

        std::cout << "\n\nleg " << leg << " angle raw data: " << std::endl;
        for (int j = 0; j < 5; j++)
        {
            std::cout << data[leg].q(j) * 180 / 3.1415 << "    ";
        }

        _biped.Joint_Remapping_lowfromhigh(leg, data[leg].q, data[leg].qd, data[leg].tau);


        std::cout << "\nleg " << leg << " angle calibrated data: " << std::endl;
        for (int j = 0; j < 5; j++)
        {
            std::cout << data[leg].q(j) * 180 / 3.1415 << "    ";
        }


        data[leg].J = _biped.HiptoFootJacobian(data[leg].q, leg);
        data[leg].J2 = data[leg].J.block(0,0, 3,5);
        data[leg].p = _biped.HiptoFoot(data[leg].q, leg);
        data[leg].v = data[leg].J2 * data[leg].qd;


    }


}








void LegController::updateCommand(LowlevelCmd* cmd){
    
    

    //For high-lev(controller-configuired) q, qd, tau, all the axis of rotations are in the same direction; +z, +x, +y.

    //LIDAR-modified
    //Necessary: To prevent actuating unused joints 


    for (int leg = 0; leg < 2; leg++){


        // //// Joint PD gains //
        double Kp_joint[5] = {10.0, 30.0, 30.0, 30.0, 10.};
                           // = {80.0, 80.0, 100.0, 120.0, 20.0};
        double Kd_joint[5] //= {1, 1.0, 1.5, 1.5, 0.5};
                           = {1.0, 1.0, 1.0, 1.0, 0.5};
                           //= {4, 4, 6, 6, 2};
        //might need to be increased



        // Decides control modes and assign inputs ======================================================================================

        if (commands[leg].which_control == 0){ // none
            JointPDSwitch = 0.0;
        
        }else if (commands[leg].which_control == 1){ // PDStand
            JointPDSwitch = 1.0;

            for (int k = 0; k < 5; k ++) {
                Kp_joint[k] = Kp_joint[k] * 1;
                Kd_joint[k] = Kd_joint[k] * 1;
                //  std::cout << "PD Standing in legctrl!!!!!!!!!!!!" << commands[leg].qDes(k) << std::endl;
                 
            }

<<<<<<< Updated upstream

        // Decides joint PD control======================================================================================

        Vec3<double> foot_des = commands[leg].pDes; 
        Vec3<double> foot_v_des = commands[leg].vDes;

        JointPDSwitch = 1.0;
        if ( foot_des(2)==0 ){ //means contact
            JointPDSwitch = 0.0;

        
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


=======
        }else if (commands[leg].which_control == 2){ // stance
            JointPDSwitch = 0.0;


        }else if (commands[leg].which_control == 3){ // swing
            JointPDSwitch = 1.0;
>>>>>>> Stashed changes

        //Doing IK to get hip roll, pitch, knee
        //with the given desired foot pos/vel and fixing the hip yaw & ankle pitch (to make it degenerated to 3 DOFs)


<<<<<<< Updated upstream
        //qDes
        commands[leg].qDes.block(1,0, 3,1) = InverseKinematics_swingctrl(foot_des, leg);
        // hip yaw = 0, ankle pitch is designed to be flat(=0)
        commands[leg].qDes(0) = 0;
        commands[leg].qDes(4) = -data[leg].q(3) - data[leg].q(2);
=======
            //qDes
            commands[leg].qDes.block(1,0, 3,1) = _biped.InverseKinematics_swingctrl(foot_des, leg);
            // hip yaw = 0, ankle pitch is designed to be flat(=0)
            commands[leg].qDes(0) = 0;
            commands[leg].qDes(4) = -data[leg].q(3) - data[leg].q(2);
>>>>>>> Stashed changes

        // qdDes
        commands[leg].qdDes = data[leg].J2.transpose() * foot_v_des;

<<<<<<< Updated upstream
=======
        }
>>>>>>> Stashed changes


        

    


        // Stance foot force to torque mapping======================================================================
        Vec6<double> footWrench =  commands[leg].feedforwardForce;
        Vec5<double> legTorque = (data[leg].J.transpose() * footWrench);
        commands[leg].tau = legTorque;


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
        motiontime++;

        std::cout<< motiontime << std::endl;

        commands[leg].tau(0) = legTorque[0] * percent;
        commands[leg].tau(1) = legTorque[1] * percent;
        commands[leg].tau(2) = legTorque[2] * percent;
        commands[leg].tau(3) = legTorque[3] * percent;
        commands[leg].tau(4) = legTorque[4] * percent;

        






        // Commands to low-level controller======================================================================

        //Command re-mapping: the reversed order of reading data
        Vec5<double> qDes_temp = commands[leg].qDes;
        Vec5<double> qdDes_temp = commands[leg].qdDes;
        Vec5<double> tau_temp = commands[leg].tau;
        _biped.Joint_Remapping_highfromlow(leg, qDes_temp, qdDes_temp, tau_temp);
        int *motor_sequence = _biped.motor_sequence;

        // Assigning to lowCmd
        for (int k = 0; k < 5; k ++) {
            cmd->motorCmd[motor_sequence[k + leg*5]].tau = tau_temp(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].q = qDes_temp(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].dq = qdDes_temp(k) * JointPDSwitch;
            cmd->motorCmd[motor_sequence[k + leg*5]].Kp = Kp_joint[k] * JointPDSwitch * percent;
            cmd->motorCmd[motor_sequence[k + leg*5]].Kd = Kd_joint[k] * 1.25;
        }

        //For unused actuators - could be removed after testing
        cmd->motorCmd[0].tau = 0;
        cmd->motorCmd[3].tau = 0;
        cmd->motorCmd[0].Kp = 0;
        cmd->motorCmd[0].Kd = 5;
        cmd->motorCmd[3].Kp = 0;
        cmd->motorCmd[3].Kd = 5;

    }


<<<<<<< Updated upstream
=======
    //This is necessary!!!!!!!!!!!!!
>>>>>>> Stashed changes
    for (int i = 0; i< 2; i++){
        commands[i].tau << 0,0,0,0,0;
        commands[i].qDes << 0,0,0,0,0;
        commands[i].qdDes << 0,0,0,0,0;
<<<<<<< Updated upstream
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
=======
>>>>>>> Stashed changes
    }
        


    
    



   
}
