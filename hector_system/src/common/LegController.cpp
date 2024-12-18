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
    kpJoint = Vec5<double>::Zero();
    kdJoint = Vec5<double>::Zero();
    kptoe = 0;
    kdtoe = 0;

    control_mode = 0;
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


    for (int i = 0; i< 2; i++){
        data[i].zero();
    }


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


    for (int leg = 0; leg < 2; leg++){


        // //// Joint PD gains //
        commands[leg].kpJoint << 10.0, 30.0, 30.0, 30.0, 10.0;
        commands[leg].kdJoint << 1.5, 1.5, 1.5, 1.5, 1;


        // ramping up for first few seconds==============================================================
        double percent;
        double duration = 4000;
        percent = (double)(motiontime-1000)/duration;
        if (motiontime < 1000) {
            percent = 0.0;
        }
        if (motiontime > 1000+duration) {
            percent = 1.0;
        }

        std::cout<< "\n motiontime is " << motiontime << std::endl;





        // Decides control modes and assign inputs ======================================================================================



        if (commands[leg].control_mode == 0){ // none

            commands[leg].kpJoint = Vec5<double>::Zero();
            commands[leg].kdJoint = Vec5<double>::Zero();

        
        }else if (commands[leg].control_mode == 1){ // PDStand

            commands[leg].kpJoint *= 1.5;
            commands[leg].kpJoint[4] *= 1.5;
            
            commands[leg].kdJoint *= 1.5;

        }else if (commands[leg].control_mode == 2){ // stance

            commands[leg].kpJoint = Vec5<double>::Zero();


            // Stabilizing the motor control and prevent jittering: Giving D target to 0 joint velocity
            commands[leg].qdDes = Vec5<double>::Zero();

            // Stance foot force to torque mapping is already done





        }else if (commands[leg].control_mode == 3){ // swing

            //Doing IK to get hip roll, pitch, knee
            //with the given desired foot pos/vel and fixing the hip yaw & ankle pitch (to make it degenerated to 3 DOFs)

            Vec3<double> foot_des = commands[leg].pDes; 
            Vec3<double> foot_v_des = commands[leg].vDes;

            //qDes
            commands[leg].qDes.block(1,0, 3,1) = _biped.InverseKinematics_swingctrl(foot_des, leg);
            // To remove 2 redundancies in DoFs (3D foot pos constraint versus 5 DoFs in each leg) -> hip yaw = 0, ankle pitch designed to be parallel to the ground
            commands[leg].qDes(0) = 0;
            commands[leg].qDes(4) = 0 -data[leg].q(3) - data[leg].q(2); // Assuming that the ground is flat

            // qdDes
            commands[leg].qdDes = data[leg].J2.transpose() * foot_v_des;

        }


        

    


        






        // Commands to low-level controller======================================================================

        //Command re-mapping: the reversed order of reading data
        Vec5<double> qDes_temp = commands[leg].qDes;
        Vec5<double> qdDes_temp = commands[leg].qdDes;
        Vec5<double> tau_temp = commands[leg].tau;
        _biped.Joint_Remapping_highfromlow(leg, qDes_temp, qdDes_temp, tau_temp);
        int *motor_sequence = _biped.motor_sequence;

        // Assigning to lowCmd
        for (int k = 0; k < 5; k ++) {
            cmd->motorCmd[motor_sequence[k + leg*5]].tau = tau_temp(k) * percent;
            cmd->motorCmd[motor_sequence[k + leg*5]].q = qDes_temp(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].dq = qdDes_temp(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].Kp = commands[leg].kpJoint[k] * percent;
            cmd->motorCmd[motor_sequence[k + leg*5]].Kd = commands[leg].kdJoint[k];
        }

        //For unused actuators - just for safety
        cmd->motorCmd[0].tau = 0;
        cmd->motorCmd[3].tau = 0;
        cmd->motorCmd[0].Kp = 0;
        cmd->motorCmd[0].Kd = 5;
        cmd->motorCmd[3].Kp = 0;
        cmd->motorCmd[3].Kd = 5;

    }

    // Necessary for stabilization. Should not go earlier than assigning inputs.
    for (int i = 0; i< 2; i++){

        commands[i].zero();
    }
        


    
   
}
