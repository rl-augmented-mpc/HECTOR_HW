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

    qDesDelta = Vec5<double>::Zero();
    feedforwardForceDelta = Vec6<double>::Zero();
    footplacementDelta = Vec2<double>::Zero();
    Pf = Vec3<double>::Zero();
    Pf_augmented = Vec3<double>::Zero();
    double contact_phase = 0; 
    double contact_state = 1; 
    double swing_phase = 0;
    double swing_state = 0;
    //control_mode should be not touched

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
        bool ConnectionIsFine = _biped.CheckMotorConnection(leg, data[leg].q);



        // ConnectionIsFine = false;
        if (!ConnectionIsFine){
            // Print the joint angles for debugging
            std::cout << "\n\nleg " << leg << " angle raw data: " << std::endl;
            for (int j = 0; j < 5; j++)
            {
                std::cout << data[leg].q(j) * 180 / 3.1415 << "    ";
            }
        }

        _biped.Joint_Remapping_lowfromhigh(leg, data[leg].q, data[leg].qd, data[leg].tau);
        // ConnectionIsFine = false;
        if (!ConnectionIsFine){
            // Print the joint angles for debugging
            std::cout << "\nleg " << leg << " angle calibrated data: " << std::endl;
            for (int j = 0; j < 5; j++)
            {
                std::cout << data[leg].q(j) * 180 / 3.1415 << "    ";
            }

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
        if (_biped._real_flag == 1){
            commands[leg].kpJoint << 10.0, 30.0, 30.0, 30.0, 10.0;
            // commands[leg].kdJoint << 1.5, 1.5, 1.5, 1.5, 1;
            commands[leg].kdJoint << 2.0, 2.0, 2.0, 2.0, 1.5;
        }
        else{
            commands[leg].kpJoint << 20.0, 20.0, 20.0, 20.0, 15.0;
            commands[leg].kdJoint << 0.45, 0.6, 0.45, 0.45, 0.6;
        }


        // ramping up for first few seconds==============================================================
        double progress;
        double duration = 4000; //4second
        double offset_time = 500; // within this time, the command is not sent
        progress = (double)(motiontime-offset_time)/duration;
        if (motiontime < offset_time) {
            progress = 0.0;
        }

        if (motiontime > offset_time+duration){
            progress = 1.0;
        }
        else{
            if (_biped._real_flag == 1){
                int barWidth = 50; 
                double percentage = progress * 100.0;
                int pos = static_cast<int>(barWidth * percentage / 100);
                std::cout << "\r" << std::string(barWidth + 10, ' ') << "\r";
                std::cout << "Gradual command increase: [";
                for (int i = 0; i < barWidth; ++i) {
                    if (i < pos)
                        std::cout << "=";
                    else if (i == pos)
                        std::cout << ">";
                    else
                        std::cout << " ";
                }
                // std::cout << "] " << percentage << "%\r"; // \r brings the cursor back to the beginning
                std::cout << "] " << std::fixed << std::setprecision(2) << percentage << "%";
                std::cout.flush(); // Ensure the output is shown immediately
            }
        }



        // Decides control modes and assign inputs ======================================================================================



        if (commands[leg].control_mode == 0){ // none

            commands[leg].kpJoint = Vec5<double>::Zero();
            commands[leg].kdJoint = Vec5<double>::Zero();

            // std::cout << "control_mode 0" << std::endl;
        
        }else if (commands[leg].control_mode == 1){ // PDStand

            commands[leg].kpJoint *= 1.5;
            commands[leg].kpJoint[4] *= 1.5;
            
            commands[leg].kdJoint *= 1.5;

            // std::cout << "control_mode 1" << std::endl;

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
            commands[leg].qDes.block(1,0, 3,1) = _biped.ComputeIK(foot_des, leg);
            // To remove 2 redundancies in DoFs (3D foot pos constraint versus 5 DoFs in each leg) -> hip yaw = 0, ankle pitch designed to be parallel to the ground
            commands[leg].qDes(0) = 0;
            // commands[leg].qDes(4) = 0 -data[leg].q(3) - data[leg].q(2); // Ankle joint parallel to torso
            commands[leg].qDes(4) = -_biped.slope_pitch - data[leg].q(3) - data[leg].q(2); // Ankle joint parallel to (sloped) ground

            // qdDes
            commands[leg].qdDes = data[leg].J2.transpose() * foot_v_des;

            // std::cout << "control_mode 3" << std::endl;

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
            if (_biped._real_flag==1){
                cmd->motorCmd[motor_sequence[k + leg*5]].tau = tau_temp(k) * progress;
                cmd->motorCmd[motor_sequence[k + leg*5]].q = qDes_temp(k);
                cmd->motorCmd[motor_sequence[k + leg*5]].dq = qdDes_temp(k);
                cmd->motorCmd[motor_sequence[k + leg*5]].Kp = commands[leg].kpJoint[k] * progress;
                cmd->motorCmd[motor_sequence[k + leg*5]].Kd = commands[leg].kdJoint[k];
            }
            else{
                // ignore torque,gain ramp up in simulation
                cmd->motorCmd[motor_sequence[k + leg*5]].tau = tau_temp(k);
                cmd->motorCmd[motor_sequence[k + leg*5]].q = qDes_temp(k);
                cmd->motorCmd[motor_sequence[k + leg*5]].dq = qdDes_temp(k);
                cmd->motorCmd[motor_sequence[k + leg*5]].Kp = commands[leg].kpJoint[k];
                cmd->motorCmd[motor_sequence[k + leg*5]].Kd = commands[leg].kdJoint[k];
            }
        }

        //For unused actuators - just for safety
        _biped.SafeGuardUnusedMotor(cmd);

    }

    // Before initialization, save
    grfm.block<6,1>(0,0) = commands[0].feedforwardForce;
    grfm.block<6,1>(6,0) = commands[1].feedforwardForce;
    qref.block<5,1>(0,0) = commands[0].qDes;
    qref.block<5,1>(5,0) = commands[1].qDes;

    // Necessary for stabilization. Should not go earlier than assigning inputs.
    for (int i = 0; i< 2; i++){
        // commands[i].zero();
        commands[i].tau.setZero();
        commands[i].feedforwardForce.setZero();
        commands[i].qDes.setZero();
        commands[i].qdDes.setZero();
        commands[i].kpJoint.setZero();
        commands[i].kdJoint.setZero();
    }  
}


// *** For RL-MPC *****************************************

Vec12<double> LegController::get_grw(){
    /// return GRF-M computed by MPC
    return grfm; 
}

Vec4<double> LegController::get_reibert_foot_placement(){
    // return the next foot placement in world frame
    Vec4<double> Pfs;
    Pfs(0) = commands[0].Pf(0);
    Pfs(1) = commands[0].Pf(1);
    Pfs(2) = commands[1].Pf(0);
    Pfs(3) = commands[1].Pf(1);
    return Pfs; 
}

Vec4<double> LegController::get_foot_placement(){
    // return the next foot placement in world frame
    Vec4<double> Pfs;
    Pfs(0) = commands[0].Pf_augmented(0);
    Pfs(1) = commands[0].Pf_augmented(1);
    Pfs(2) = commands[1].Pf_augmented(0);
    Pfs(3) = commands[1].Pf_augmented(1);
    return Pfs; 
}

Vec6<double> LegController::get_ref_swing_position(){
    // return reference foot position in body frame
    Vec6<double> foot_ref_pos;
    foot_ref_pos.block<3,1>(0,0) = commands[0].pDes; 
    foot_ref_pos.block<3,1>(3,0) = commands[1].pDes; 
    return foot_ref_pos; 
}

Vec6<double> LegController::get_swing_position(){
    // return foot position in body frame
    Vec6<double> foot_pos; 
    foot_pos.block<3,1>(0,0) = data[0].p + _biped.getHip2Location(0);
    foot_pos.block<3,1>(3,0) = data[1].p + _biped.getHip2Location(1);
    return foot_pos;
}

Vec2<double> LegController::get_contact_phase(){
    // return contact phase of the legs
    Vec2<double> contact_phase;
    contact_phase(0) = commands[0].contact_phase * commands[0].contact_state;
    contact_phase(1) = commands[1].contact_phase * commands[1].contact_state;
    return contact_phase;
}

Vec2<double> LegController::get_swing_phase(){
    // return swing phase of the legs
    Vec2<double> swing_phase;
    swing_phase(0) = commands[0].swing_phase * commands[0].swing_state;
    swing_phase(1) = commands[1].swing_phase * commands[1].swing_state;
    return swing_phase;
}

Vec2<double> LegController::get_contact_state(){
    // return contact state of the legs
    Vec2<double> contact_state;
    contact_state(0) = commands[0].contact_state;
    contact_state(1) = commands[1].contact_state;
    return contact_state;
}

Vec2<double> LegController::get_swing_state(){
    // return swing state of the legs
    Vec2<double> swing_state;
    swing_state(0) = commands[0].swing_state;
    swing_state(1) = commands[1].swing_state;
    return swing_state;
}



void LegController::update_grw(Vec12<double> _grfm){
    for (int i = 0; i < 2; i++){
        commands[i].feedforwardForce = _grfm.block<6,1>(i*6,0);
    }
}

void LegController::add_residual_grw(Vec12<double> delta_grfm){
    for (int i = 0; i < 2; i++){
        commands[i].feedforwardForceDelta = delta_grfm.block<6,1>(i*6,0);
    }
}

void LegController::add_residual_joint_position(Vec10<double> delta_joint_position){
    for (int i = 0; i < 2; i++){
        commands[i].qDesDelta = delta_joint_position.block<5,1>(i*5,0);
    }
}

void LegController::updateCommandResidual(LowlevelCmd* cmd){
    // update legtau with augmented GRFM
    for (int i = 0; i < 2; i++){
        Vec6<double> footForce = grfm.block<6,1>(i*6,0) + commands[i].feedforwardForceDelta; // augmented GRF-M
        Vec5<double> legtau = data[i].J.transpose() * footForce; // force moment from stance leg
        Vec5<double> qDes = qref.block<5,1>(i*5,0) + commands[i].qDesDelta; // qref is the reference joint position

        // Push the commands to the low level message
        for (int j = 0; j < 5; j++){
            cmd->motorCmd[i*5+j].tau = legtau(j);
            cmd->motorCmd[i*5+j].q = qDes(j);
        }
        
    }
}