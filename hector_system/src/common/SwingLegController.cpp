#include "../../include/common/SwingLegController.h"

/******************************************************************************************************/
/******************************************************************************************************/
swingLegController::swingLegController():
    lip_controller(0.55){
        // updateFootPosition();
        // Pf_world[0].setZero();
        // Pf_world[1].setZero();
        // Pf_base[0].setZero();
        // Pf_base[1].setZero();
    }

void swingLegController::initSwingLegController(ControlFSMData *_data, Gait *_gait, double dtSwing)
{
    data = _data;
    gait = _gait;
    _dtSwing = dtSwing;
    updateFootPosition();

    for (int foot=0; foot < nLegs; foot++){
        if (data->_biped->swing_foot_reference_frame == "world"){
            Pf_world[foot] = pFoot_w[foot];
            Pf_base[foot] = data->_legController->data[foot].p;
        }
        else if (data->_biped->swing_foot_reference_frame == "base"){
            Pf_world[foot] = pFoot_w[foot];
            Pf_base[foot] = data->_legController->data[foot].p;
        }
    }
}

void swingLegController::setGait(Gait *_gait){
    gait = _gait;
}

void swingLegController::updateFootPlacementPlanner(){
    seResult = data->_stateEstimator->getResult();
    updateSwingStates();
    updateFootPosition();
    updateSwingTimes();
}

void swingLegController::updateSwingFootCommand(){
    computeFootDesiredPosition();
    setDesiredJointState();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){
        pFoot_w[i] =  seResult.position + seResult.rBody.transpose()*(data->_legController->data[i].p);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingStates(){
    swingStates = gait->getSwingSubPhase();
    contactStates = gait->getContactSubPhase();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingTimes(){
    for(int leg = 0; leg < nLegs; leg++){
        if(firstSwing[leg]){
            swingTimes[leg] = gait->_swing_durations_sec[leg];
        }
        else{
            swingTimes[leg] -= _dtSwing;
            if(contactStates[leg] > 0){
                firstSwing[leg] = true;
            }            
        }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootPlacement(){

    auto &stateCommand = data->_desiredStateCommand;
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    Vec3<double> v_des_world;
    v_des_world = seResult.rBody.transpose() * v_des_robot;
    Vec3<double> omega_des_world{0, 0, stateCommand->data.stateDes[11]};

    if (plannar == "LIP"){
        // 3D LIP Model
        for(int foot = 0; foot < nLegs; foot++){
            if(swingStates[foot] >= 0){
                if (firstSwing[foot]){ // computes only once at the beginning of the swing phase
                    lip_controller.update_stance_leg(1-foot, pFoot_w[1-foot].block<2,1>(0,0));
                }

                lip_controller.update_swing_times(swingTimes[foot], gait->_swing_durations_sec[foot]);
                lip_controller.compute_icp_init(seResult);
                lip_controller.compute_icp_final();
                lip_foot_placement = lip_controller.compute_foot_placement(seResult, stateCommand->data, Vec2<double>{0.0, 0.0});
                Pf_world[foot] << lip_foot_placement[0], lip_foot_placement[1], data->_biped->pf_z;

            }
            // else{
            //     Pf[foot] = pFoot_w[foot];
            // }

            footSwingTrajectory[foot].setHeight(data->_biped->foot_height);
            footSwingTrajectory[foot].setFinalPosition(Pf_world[foot]);

        }
    }

    else if (plannar == "Raibert"){
        // // Raibert heuristic
        for(int foot = 0; foot < nLegs; foot++){
            if(swingStates[foot] >= 0){ // in swing

                // Raibert heuristic: Pf = p_hip + v * \Delta{t}/2 + k * (v - v_ref)
                // eq.7.4 https://www.ri.cmu.edu/pub_files/pub3/raibert_marc_h_1983_1/raibert_marc_h_1983_1.pdf
                Vec3<double> hip_pos = seResult.position + seResult.rBody.transpose() * (data->_biped->get_hip_offset(foot));

                // // USC original (prob wrong?)
                // Pf[foot] = hip_pos + seResult.vWorld * swingTimes[foot];
                Pf_world[foot] = hip_pos + 0.5 * seResult.vWorld * swingTimes[foot];

                // feedback correction term
                double p_rel_max_x = 0.3;
                double p_rel_max_y =  0.3;
                double k_x = 0.03; 
                double k_y = 0.03;

                // // USC original (prob wrong?)
                // double pfx_rel = 0.5*seResult.vWorld[0] * gait->_swing_durations_sec[foot] + k_x * (seResult.vWorld[0] - v_des_world[0]);
                // double pfy_rel = 0.5*seResult.vWorld[1] * gait->_swing_durations_sec[foot] + k_y  * (seResult.vWorld[1] - v_des_world[1]);

                // Raibert feedback correction from original paper
                double pfx_rel = k_x  * (seResult.vWorld[0] - v_des_world[0]);
                double pfy_rel = k_y  * (seResult.vWorld[1] - v_des_world[1]);

                pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max_x), p_rel_max_x);
                pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max_y), p_rel_max_y);
                
                Pf_world[foot][0] = Pf_world[foot][0] + pfx_rel;
                Pf_world[foot][1] = Pf_world[foot][1] + pfy_rel; 
                Pf_world[foot][2] = data->_biped->pf_z;

                if (data->_biped->swing_foot_reference_frame == "world"){
                    Pf_world[foot] = Pf_world[foot];
                    Pf_base[foot] = seResult.rBody * (Pf_world[foot] - seResult.position);
                }
                else if (data->_biped->swing_foot_reference_frame == "base"){
                    Pf_base[foot] = seResult.rBody * (Pf_world[foot] - seResult.position);
                    // foothold z position is same as initial foot z pos
                    Pf_base[foot][2] = footSwingTrajectory[foot].getInitialPosition()[2];
                }

                // block nan 
                if (Pf_world[foot].hasNaN()){
                    Pf_world[foot] << 0, 0, 0;
                }
                if (Pf_base[foot].hasNaN()){
                    Pf_base[foot] << 0, 0, 0;
                }
            }

            else{ // in stance
                if (data->_biped->swing_foot_reference_frame == "world"){
                    // Pf_world[foot] = Pf_world[foot];
                    Pf_base[foot] = seResult.rBody * (Pf_world[foot] - seResult.position);
                }
                else if (data->_biped->swing_foot_reference_frame == "base"){
                    // recompute foothold for stance leg with the current base pose
                    Pf_base[foot] = seResult.rBody * (Pf_world[foot] - seResult.position);
                    // z position same as initial foot z pos
                    Pf_base[foot][2] = footSwingTrajectory[foot].getInitialPosition()[2];
                }

                // block nan 
                if (Pf_world[foot].hasNaN()){
                    // std::cout << "has nan Pf_world!" << std::endl;
                    Pf_world[foot] << 0, 0, 0;
                }
                if (Pf_base[foot].hasNaN()){
                    // std::cout << "has nan Pf_base!" << std::endl;
                    Pf_base[foot] << 0, 0, 0;
                }
            }

            footSwingTrajectory[foot].setHeight(data->_biped->foot_height);

            if (data->_biped->swing_foot_reference_frame == "world"){
                footSwingTrajectory[foot].setFinalPosition(Pf_world[foot]);
            }
            else if (data->_biped->swing_foot_reference_frame == "base"){
                footSwingTrajectory[foot].setFinalPosition(Pf_base[foot]);
            }
        }
    }
}

void swingLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        if(swingStates[foot] >= 0){ // swing phase
            if (firstSwing[foot]){
                firstSwing[foot] = false;
                if (data->_biped->swing_foot_reference_frame == "world"){
                    footSwingTrajectory[foot].setInitialPosition(pFoot_w[foot]);
                }
                else if (data->_biped->swing_foot_reference_frame == "base"){
                    footSwingTrajectory[foot].setInitialPosition(data->_legController->data[foot].p);
                }
            }

            //Compute and get the desired foot position and velocity
            footSwingTrajectory[foot].setControlPointCoef(data->_biped->cp1_coef, data->_biped->cp2_coef);
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(
                swingStates[foot], 
                gait->_swing_durations_sec[foot]
            );

            if (data->_biped->swing_foot_reference_frame == "world"){
                Vec3<double> pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
                Vec3<double> vDesFootWorld = footSwingTrajectory[foot].getVelocity().cast<double>();
                pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position);
                vFoot_b[foot] = seResult.rBody * (vDesFootWorld - seResult.vWorld);
            }
            else if (data->_biped->swing_foot_reference_frame == "base"){
                pFoot_b[foot] = footSwingTrajectory[foot].getPosition().cast<double>();
                vFoot_b[foot] = footSwingTrajectory[foot].getVelocity().cast<double>();
            }
        }
        else{ // stance phase
            if (data->_biped->swing_foot_reference_frame == "world"){
                pFoot_b[foot] = seResult.rBody * (Pf_world[foot] - seResult.position);
                vFoot_b[foot] = seResult.rBody * (Pf_world[foot] * 0 - seResult.vWorld);
            }
            else if (data->_biped->swing_foot_reference_frame == "base"){
                pFoot_b[foot] = Pf_base[foot];
                vFoot_b[foot] = Pf_base[foot] * 0;
            }
        }

        if (pFoot_b[foot].hasNaN()){
            pFoot_b[foot] << 0, 0, 0;
        }
        if (vFoot_b[foot].hasNaN()){
            vFoot_b[foot] << 0, 0, 0;
        }
    } 
}

void swingLegController::setDesiredJointState(){
    for(int leg = 0; leg < nLegs; leg++){
        if(swingStates[leg] >= 0){
            // ** joint space PD **
            data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
            data->_legController->commands[leg].tau << 0, 0, 0, 0, 0; 
            data->_legController->commands[leg].pDes = pFoot_b[leg];
            data->_legController->commands[leg].vDes = vFoot_b[leg];
            data->_legController->commands[leg].control_mode = int(ControlMode::SWING);
        }
        else{
            data->_legController->commands[leg].pDes = pFoot_b[leg];
            data->_legController->commands[leg].vDes = vFoot_b[leg]; 
        }
    }
}


std::array<Vec3<double>, 10> swingLegController::getReferenceSwingFootPosition(){
    std::array<Vec3<double>, 10> refSwingFootPosition;
    double phase = 0.0;
    double phase_increment = 0.11; 
    for (int i = 0; i < 10; i++){
        for (int leg = 0; leg < nLegs; leg++){
            if (swingStates[leg] >= 0){
                footSwingTrajectory[leg].computeSwingTrajectoryBezier(
                    phase, 
                    gait->_swing_durations_sec[leg]
                );
                refSwingFootPosition[i] = footSwingTrajectory[leg].getPosition().cast<double>();
                phase += phase_increment;
            }
        }

        if (contactStates[0] >= 0 && contactStates[1] >= 0){
            if (data->_biped->swing_foot_reference_frame == "world"){
                refSwingFootPosition[i] = pFoot_w[0]; 
            }
            else if (data->_biped->swing_foot_reference_frame == "base"){
                refSwingFootPosition[i] = data->_legController->data[0].p;
            }
        }
    }
    return refSwingFootPosition;
}

// ======================
// residual footplacement

void swingLegController::setFootplacementResidual(Vec2<double> pf_residual, int foot){
    Pf_residual[foot] = pf_residual;
}

Vec3<double> swingLegController::get_foot_placement_in_world(int foot){
    return Pf_world[foot];
}

Vec3<double> swingLegController::get_foot_placement_in_base(int foot){
    return Pf_base[foot];
}

// Vec3<double> swingLegController::getAugmentedFootPlacement(int foot){
//     return Pf_augmented[foot];
// }