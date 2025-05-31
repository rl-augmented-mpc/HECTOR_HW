#include "../../include/common/SwingLegController.h"

/******************************************************************************************************/
/******************************************************************************************************/
swingLegController::swingLegController():
    lip_controller(0.55){}

void swingLegController::initSwingLegController(ControlFSMData *_data, Gait *_gait, double dtSwing)
{
    data = _data;
    gait = _gait;
    _dtSwing = dtSwing;
    updateFootPosition();
}

void swingLegController::setGait(Gait *_gait){
    gait = _gait;
}

void swingLegController::updateFootPlacementPlanner(){
    seResult = data->_stateEstimator->getResult();
    updateSwingStates();
    updateFootPosition();
    updateSwingTimes();
    computeFootPlacement();     
}

void swingLegController::updateSwingFootCommand(){
    computeFootDesiredPosition();
    setDesiredJointState();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){
        // pFoot_w[i] =  seResult.position + seResult.rBody.transpose() 
        //             * ( data->_biped->getHip2Location(i) + data->_legController->data[i].p);
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
            // swingTimes[leg] = _dtSwing * gait->_swing(leg);
            swingTimes[leg] = gait->_swing_durations_sec[leg];
        }
        else{
            // swingTimes[leg] -= _stepping_frequency * _dtSwing;
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

                // LIP for sagittal
                lip_controller.update_swing_times(swingTimes[foot], gait->_swing_durations_sec[foot]);
                lip_controller.compute_icp_init(seResult);
                lip_controller.compute_icp_final();
                lip_foot_placement = lip_controller.compute_foot_placement(seResult, stateCommand->data, Vec2<double>{0.0, 0.0});
                Pf[foot] << lip_foot_placement[0], lip_foot_placement[1], data->_biped->pf_z;

                // Reibert for lateral
                Vec3<double>rb_fps = seResult.position + seResult.rBody.transpose() * (data->_biped->get_hip_offset(foot)) + seResult.vWorld * swingTimes[foot];
                double p_rel_max_y =  0.3;
                double k_y = 0.1;
                double pfy_rel   =  k_y  * (seResult.vWorld[1] - v_des_world[1]);
                pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max_y), p_rel_max_y);

                Pf[foot] << lip_foot_placement[0], rb_fps[1] + pfy_rel, data->_biped->pf_z;
                Pf_augmented[foot] << Pf[foot][0] + Pf_residual[foot][0], Pf[foot][1] + Pf_residual[foot][1], data->_biped->pf_z;

            }
            else{
                Pf[foot] << pFoot_w[foot].block<2,1>(0,0);
                Pf_augmented[foot] = Pf[foot];
            }

            footSwingTrajectory[foot].setHeight(data->_biped->foot_height);
            footSwingTrajectory[foot].setFinalPosition(Pf_augmented[foot]);

        }
    }

    else if (plannar == "Raibert"){
        // // Raibert heuristic
        for(int foot = 0; foot < nLegs; foot++){
            if(swingStates[foot] >= 0){

                // Reibert heuristic
                // Pf[foot] = seResult.position + seResult.rBody.transpose() * (data->_biped->getHip2Location(foot)) + seResult.vWorld * swingTimes[foot];
                Pf[foot] = seResult.position + seResult.rBody.transpose() * (data->_biped->get_hip_offset(foot)) + seResult.vWorld * swingTimes[foot];
                
                double p_rel_max_x = 0.3;
                double p_rel_max_y =  0.03;
                double k_x = 0.03; 
                double k_y = 0.03; // IMOPRTANT parameter for stable lateral motion
                
                double pfx_rel   =  seResult.vWorld[0] * 0.5 * gait->_swing_durations_sec(foot) + k_x  * (seResult.vWorld[0] - v_des_world[0]);
                double pfy_rel   =  seResult.vWorld[1] * 0.5 * gait->_swing_durations_sec(foot) + k_y  * (seResult.vWorld[1] - v_des_world[1]);
                pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max_x), p_rel_max_x);
                pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max_y), p_rel_max_y);

                Pf[foot][0] += pfx_rel;
                Pf[foot][1] += pfy_rel; 
                Pf[foot][2] = data->_biped->pf_z;

                // footplacement from reibert heuristic and residual learning
                Pf_augmented[foot][0] = Pf[foot][0] + Pf_residual[foot][0];
                Pf_augmented[foot][1] = Pf[foot][1] + Pf_residual[foot][1];
            }
            else{
                Pf[foot] << pFoot_w[foot].block<2,1>(0,0);
                Pf[foot][2] = 0.0; 
                Pf_augmented[foot] = Pf[foot];
            }

            footSwingTrajectory[foot].setHeight(data->_biped->foot_height);
            footSwingTrajectory[foot].setPitch(data->_biped->slope_pitch); 
            footSwingTrajectory[foot].setFinalPosition(Pf_augmented[foot]);   
        }
    }

    else if (plannar == "OpenLoop"){
        // Open loop gait + raibert for lateral direction 
        for(int foot = 0; foot < nLegs; foot++){
            footSwingTrajectory[foot].setHeight(data->_biped->foot_height);
            footSwingTrajectory[foot].setPitch(data->_biped->slope_pitch); 

            // Reibert heuristic
            Vec3<double> rbf = seResult.position + seResult.rBody.transpose() * (data->_biped->getHip2Location(foot)) + seResult.vWorld * swingTimes[foot];
            Vec3<double> hip_pos = seResult.position + seResult.rBody.transpose() * (data->_biped->getHip2Location(foot));
            if (firstSwing[foot]){
                float px = v_des_robot[0]*swingTimes[foot];
                Pf[foot][0] = hip_pos[0]+ px;
                Pf[foot][1] = rbf[1];
            }
            else{
                Pf[foot][1] = rbf[1];
            }

            Pf[foot][2] = 0.0;

            // footplacement from reibert heuristic and residual learning
            Pf_augmented[foot][0] = Pf[foot][0];
            Pf_augmented[foot][1] = Pf[foot][1];
            footSwingTrajectory[foot].setFinalPosition(Pf_augmented[foot]);   
        }
    }
}


/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        if(swingStates[foot] >= 0){
            if (firstSwing[foot]){
              firstSwing[foot] = false;
              footSwingTrajectory[foot].setInitialPosition(pFoot_w[foot]);
               }
            //Compute and get the desired foot position and velocity
            footSwingTrajectory[foot].setControlPointCoef(data->_biped->cp1_coef, data->_biped->cp2_coef);
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(
                swingStates[foot], 
                gait->_swing_durations_sec[foot]
            );
            Vec3<double> pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
            Vec3<double> vDesFootWorld = footSwingTrajectory[foot].getVelocity().cast<double>();
            
            pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position);
            vFoot_b[foot] = seResult.rBody * (vDesFootWorld - seResult.vWorld);  
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
            data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
            data->_legController->commands[leg].tau << 0, 0, 0, 0, 0; 
            data->_legController->commands[leg].pDes = pFoot_b[leg];
            data->_legController->commands[leg].vDes = vFoot_b[leg];           
            data->_legController->commands[leg].kptoe = 10; // not used
            data->_legController->commands[leg].kdtoe = 0.2; // not used
            data->_legController->commands[leg].control_mode = int(ControlMode::SWING);
        }
    }
}

// ======================
// residual footplacement

void swingLegController::setFootplacementResidual(Vec2<double> pf_residual, int foot){
    Pf_residual[foot] = pf_residual;
}

Vec3<double> swingLegController::getReibertFootPlacement(int foot){
    return Pf[foot];
}

Vec3<double> swingLegController::getAugmentedFootPlacement(int foot){
    return Pf_augmented[foot];
}