#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"

using namespace ori;

ConvexMPCLocomotion::ConvexMPCLocomotion(
  double _dt, 
  int _iterations_between_mpc, 
  int _horizon_length, 
  int _mpc_decimation) : 
 iterationsBetweenMPC(_iterations_between_mpc),
 horizonLength(_horizon_length),
 dt(_dt),
 mpc_decimation(_mpc_decimation),
 standing(horizonLength, Vec2<int>(5, 5), Vec2<int>(0, 0), dt, iterationsBetweenMPC*dt), // set random dsp duration b/c stance gait is non-periodic, infinite
 walking(horizonLength, Vec2<int>(0, 0), Vec2<int>(5, 5), dt, iterationsBetweenMPC*dt) // set nominal value at the beginning. You can change parameter through updateGait
{
  dtMPC = dt * iterationsBetweenMPC;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;
}

void ConvexMPCLocomotion::updateGait(Vec2<int> dsp_durations, Vec2<int> ssp_durations)
{
  standing.update_parameter(Vec2<int>(5, 5), Vec2<int>(0, 0));
  walking.update_parameter(dsp_durations, ssp_durations);
  walking.reset();
  standing.reset();
}

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
  if (data._biped->reset_gait)
  {
    updateGait(data._biped->dsp_durations, data._biped->ssp_durations);
    data._biped->reset_gait = false;
  }
  Gait *gait = &standing;
  if (gaitNumber == 2)
    gait = &walking;
  else if (gaitNumber == 1)
    gait = &standing;

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() 
    * (data._biped->getHip2Location(i) + data._legController->data[i].p);
    // pFoot[i] = seResult.position + seResult.rBody.transpose()*(data._legController->data[i].p);
  }

  // some first time initialization
  if (firstRun)
  {
    swing.initSwingLegController(&data, gait, dt);
    swing.setPlanner(data._biped->foot_placement_planner);
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    yaw_desired = seResult.rpy[2];
    firstRun = false;
  }

  // set command velocity
  v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], stateCommand->data.stateDes[8];
  v_des_world = seResult.rBody.transpose() * v_des_robot;
  turn_rate_des = stateCommand->data.stateDes[11];

  if (gaitNumber==1){
    v_des_robot << 0, 0, 0;
    v_des_world << 0, 0, 0;
    turn_rate_des = 0;
  }

  // foot placement planner
  swing.setGait(gait);
  swing.setFootHeight(data._biped->foot_height);
  swing.setSteppingFrequency((double)data._biped->gait_stepping_frequency);
  swing.setFootplacementResidual(data._biped->rl_params.delta_foot_placement.segment(0,2), 0);
  swing.setFootplacementResidual(data._biped->rl_params.delta_foot_placement.segment(2,2), 1);
  swing.updateFootPlacementPlanner();

  // load LCM leg swing gains
  Kp << 250, 0, 0,
      0, 250, 0,
      0, 0, 200;
  Kp_stance = 0* Kp;

  Kd << 5, 0, 0,
      0, 5, 0,
      0, 0, 5;
  Kd_stance = 0*Kd;

  Vec2<double> contactStates = gait->getContactSubPhase();
  Vec2<double> swingStates = gait->getSwingSubPhase();

  if (contactStates(0) == -1 && contactStates(1) == -1){
    std::cout << "something wrong with contact state" << std::endl;
  }

  // update MPC
  if (iterationCounter % mpc_decimation == 0){
    int *mpcTable = gait->mpc_gait();
    updateMPC(mpcTable, data, omniMode);
    data._biped->update_mpc_cost(qp_cost);
  }

  // =========================
  // update swing foot command
  // =========================
  swing.updateSwingFootCommand();

  // ==========================
  // update stance foot command
  // ==========================
  Vec2<double> se_contactState(0, 0);
  for (int foot = 0; foot < 2; foot++)
  {

    if (swingStates(foot) >= 0) // foot is in swing
    {
      se_contactState[foot] = contactStates[foot];
    }

    else if (contactStates(foot) >= 0) // foot is in stance
    { 
      Vec3<double> pDesLeg = {0, 0, 0};
      Vec3<double> vDesLeg = {0, 0, 0};
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;

      data._legController->commands[foot].kptoe = 0; // not used
      data._legController->commands[foot].kdtoe = 0; // not used

      data._legController->commands[foot].feedforwardForce = f_ff[foot];
      data._legController->commands[foot].tau = data._legController->data[foot].J.transpose()* f_ff[foot];
      data._legController->commands[foot].control_mode = int(ControlMode::STANCE);
      se_contactState[foot] = contactStates(foot);
    }
    data._stateEstimator->setContactPhase(se_contactState);
  }

  // // update sampling time when contact switch happens
  // // gait->updateSamplingTime(data._biped->rl_params._dt_sampling);
  // if ((swingStates(0) == -1 && swing_states_prev(0) != -1) || 
  // (swingStates(1) == -1 && swing_states_prev(1) != -1) || 
  // (swing_states_prev(0)==0 && swing_states_prev(1)==0)){
  //   gait->updateSamplingTime(data._biped->rl_params._dt_sampling);
  // }
  // swing_states_prev = swingStates;
  
  // update every control time step
  gait->updateSamplingTime(data._biped->rl_params._dt_sampling);
  
  gait->updatePhase();
  iterationCounter++;

  // update contact state with incremented gait phase
  contactStates = gait->getContactSubPhase();
  swingStates = gait->getSwingSubPhase();
  for (int foot = 0; foot < 2; foot++)
  {
    // push back data to leg controller
    double contactState = contactStates(foot);
    double swingState = swingStates(foot);
    Vec3<double> raibert_footplacement = swing.getReibertFootPlacement(foot);
    Vec3<double> augmented_footplacement = swing.getAugmentedFootPlacement(foot);

    data._legController->commands[foot].Pf = raibert_footplacement;
    data._legController->commands[foot].Pf_augmented = augmented_footplacement;

    data._legController->commands[foot].contact_phase = contactState;
    data._legController->commands[foot].swing_phase = swingState;

    if (contactState >= 0){
      data._legController->commands[foot].contact_state = 1.;
    }
    else{
      data._legController->commands[foot].contact_state = 0.;
    }
    if (swingState >= 0){
      data._legController->commands[foot].swing_state = 1.;
    }
    else{
      data._legController->commands[foot].swing_state = 0.;
    }

  }
}

void ConvexMPCLocomotion::updateMPC(int *mpcTable, ControlFSMData &data, bool omniMode)
{
  auto seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  double *p = seResult.position.data();
  double *v = seResult.vWorld.data();
  double *w = seResult.omegaWorld.data();
  double *q = seResult.orientation.data();
  double yaw = seResult.rpy[2];

  double r[6];
  for (int i = 0; i < 6; i++)
  {
    r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
  }

  // roll pitch yaw x y z droll dpitch dyaw dx dy dz
  // double Q[12] = {300, 300, 150,   300, 300, 100,   1, 1, 1,   5, 3, 3}; // original hardware
  // double Q[12] = {100, 200, 300,  300, 300, 300,  1, 1, 3.0,  2.0, 2.0, 1};
  // double Q[12] = {100, 100, 500,  100, 100, 100,  1, 1, 5,  5, 5, 1};
  double Q[12] = {150, 150, 250,  500, 500, 500,  1, 1, 5,  1, 1, 1}; //best weight

  // double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   2e-2, 2e-2, 2e-2, 2e-2, 2e-2, 2e-2}; // original hardware
  // double Alpha[12] = {2e-4, 2e-4, 2e-4, 2e-4, 2e-4, 2e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
  double Alpha[12] = {1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5,   1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4}; // best weight

  double *weights = Q;
  double *Alpha_K = Alpha;

  updateReferenceTrajectory(seResult, *stateCommand);

  // dtMPC = dt * iterationsBetweenMPC;
  dtMPC = data._biped->rl_params._dt_sampling;
  setup_problem(
  dtMPC, horizonLength, data._biped->mu, data._biped->f_max, data._biped->mass, 
  data._biped->I_body, data._biped->rl_params.A_residual, data._biped->rl_params.B_residual);
  
  update_problem_data(p, v, q, w, r, yaw, weights, trajAll, Alpha_K, mpcTable, data);

  for (int leg = 0; leg < 2; leg++)
  {
    Vec3<double> GRF;
    Vec3<double> GRF_R;
    Vec3<double> GRM;
    Vec3<double> GRM_R;
    Vec6<double> f;
    for (int axis = 0; axis < 3; axis++)
    {

      GRF[axis] = get_solution(leg * 3 + axis);
      GRM[axis] = get_solution(leg * 3 + axis + 6);
    }
    // std::cout << "GRF: " << GRF.transpose() << std::endl;
    GRF_R = -seResult.rBody * GRF;
    GRM_R = -seResult.rBody * GRM;

    for (int i = 0; i < 3; i++){
      f(i) = GRF_R(i);
      f(i+3) = GRM_R(i);
    }
    f_ff[leg] = f;
  }
  contact_state(0) = mpcTable[0];
  contact_state(1) = mpcTable[1];
  
  qp_cost = get_cost();
}


void ConvexMPCLocomotion::updateReferenceTrajectory(StateEstimate &seResult, DesiredStateCommand &stateCommand){
  // planar condition

  // reset yaw knot point (to deal with euler angle singularity)
  double angle_eps = M_PI/36;
  if (yaw_desired > M_PI-angle_eps || yaw_desired < -M_PI+angle_eps){
    yaw_desired = seResult.rpy[2];
  }

  // Gait *gait = &standing;
  // if (gaitNumber == 2)
  //   gait = &walking;
  // else if (gaitNumber == 1)
  //   gait = &standing;
  // Vec2<double> swingStates = gait->getSwingSubPhase();

  // // SSP: do not move
  // if (swingStates(0) >=0 | swingStates(1) >= 0){
  //   v_des_world[0] = 0;
  //   v_des_world[1] = 0;
  //   v_des_world[2] = 0;
  //   turn_rate_des = 0;
  // }

  world_position_desired[0] += mpc_decimation*dt * v_des_world[0];
  world_position_desired[1] += mpc_decimation*dt * v_des_world[1];
  world_position_desired[2] = stateCommand.data.stateDes[2];
  yaw_desired += mpc_decimation*dt * turn_rate_des;

  stateCommand.data.stateDes[0] = world_position_desired[0];
  stateCommand.data.stateDes[1] = world_position_desired[1];
  stateCommand.data.stateDes[5] = yaw_desired;

  // if current position deviates from reference too much, reset reference close to current pose
  const double max_pos_error = 0.1;
  const double max_yaw_error = 0.3;
  double xStart = world_position_desired[0];
  double yStart = world_position_desired[1];
  double yawStart = yaw_desired;
  if(xStart - seResult.position[0] > max_pos_error){
    xStart = seResult.position[0] + max_pos_error;
  }
  if(seResult.position[0] - xStart > max_pos_error){
    xStart = seResult.position[0] - max_pos_error;
  }
  if(yStart - seResult.position[1] > max_pos_error){
    yStart = seResult.position[1] + max_pos_error;
  }
  if(seResult.position[1] - yStart > max_pos_error){
    yStart = seResult.position[1] - max_pos_error;
  }
  if(yawStart - seResult.rpy[2] > max_yaw_error){
    yawStart = seResult.rpy[2] + max_yaw_error;
  }
  if(seResult.rpy[2] - yawStart > max_yaw_error){
    yawStart = seResult.rpy[2] - max_yaw_error;
  }
  double trajInitial[12] = {stateCommand.data.stateDes[3],  // roll
                            stateCommand.data.stateDes[4],   // pitch
                            yawStart, // yaw
                            xStart, // x
                            yStart, // y
                            world_position_desired[2], // z
                            0, // wx
                            0, // wy
                            turn_rate_des,  // wz
                            v_des_world[0], // vx
                            v_des_world[1], // vy
                            v_des_world[2]};   // vz

  // get trajectory though mpc horizon
  for (int i = 0; i < horizonLength; i++)
  {
    for (int j = 0; j < 12; j++)
      trajAll[12 * i + j] = trajInitial[j];

    // trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0];
    // trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1];
    // trajAll[12*i + 2] = seResult.rpy[2] + i * dtMPC * turn_rate_des;

    // blend closed-loop and open-loop trajectory
    double alpha = 0.75;
    trajAll[12*i + 3] = alpha * (seResult.position[0] + i * dtMPC * v_des_world[0])
                        + (1 - alpha) * (trajInitial[3] + i * dtMPC * v_des_world[0]);

    trajAll[12*i + 4] = alpha * (seResult.position[1] + i * dtMPC * v_des_world[1])
                        + (1 - alpha) * (trajInitial[4] + i * dtMPC * v_des_world[1]);
    
    trajAll[12*i + 2] = alpha * (seResult.rpy[2] + i * dtMPC * turn_rate_des)
                        + (1 - alpha) * (trajInitial[2] + i * dtMPC * turn_rate_des);

    // if velocity is too small, use open-loop trajectory
    if (std::abs(v_des_world[0]) < 0.01){
      trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
    }

    if (std::abs(v_des_world[1]) < 0.01){
      trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
    }

    if (std::abs(turn_rate_des) < 0.01){
      trajAll[12*i + 2] = trajInitial[2] + i * dtMPC * turn_rate_des;
    }
  }
}