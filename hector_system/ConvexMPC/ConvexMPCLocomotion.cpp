#include <iostream>
#include "../include/common/Utilities/Timer.h"
// #include "../include/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
// #include "../include/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
// #include "../include/body.h"

using namespace ori;
// using namespace laikago_model;

/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                                    horizonLength(10),
                                                                                    dt(_dt),
                                                                                    standing(horizonLength, Vec2<int>(int(0.2/_dt), int(0.2/_dt)), Vec2<int>(0, 0)), 
                                                                                    walking(horizonLength, Vec2<int>(int(0.0/_dt), int(0.0/_dt)), Vec2<int>(int(0.3/dt), int(0.3/_dt)))
{
  dtMPC = dt * iterationsBetweenMPC;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;
}

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
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
  }

  // some first time initialization
  if (firstRun)
  {
    swing.initSwingLegController(&data, gait, dt);
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    yaw_desired = seResult.rpy[2];
    firstRun = false;
  }

  // set command velocity
  v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0.0;
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

  // update gait phase
  gait->updatePhase(data._biped->gait_stepping_frequency);

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

  // construct contact constraint booleans for MPC
  int *mpcTable = gait->mpc_gait(iterationsBetweenMPC);
  if (iterationCounter % 5 == 0){
    updateMPC(mpcTable, data, omniMode);
  }
  iterationCounter++;

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

    if (swingStates(foot) > 0) // foot is in swing
    {
      se_contactState[foot] = contactStates[foot];
    }

    else if (contactStates(foot) > 0) // foot is in stance
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

    // push back data to leg controller
    // double contactState = contactStates(foot);
    // double swingState = swingStates(foot);
    // Vec3<double> reibert_footplacement = swing.getReibertFootPlacement(foot);
    // Vec3<double> augmented_footplacement = swing.getAugmentedFootPlacement(foot);

    // data._legController->commands[foot].Pf = reibert_footplacement;
    // data._legController->commands[foot].Pf_augmented = augmented_footplacement;

    // data._legController->commands[foot].contact_phase = contactState;
    // data._legController->commands[foot].swing_phase = swingState;

    // if (contactState > 0){
    //   data._legController->commands[foot].contact_state = 1;
    // }
    // else{
    //   data._legController->commands[foot].contact_state = 0;
    // }
    // if (swingState > 0){
    //   data._legController->commands[foot].swing_state = 1;
    // }
    // else{
    //   data._legController->commands[foot].swing_state = 0;
    // }

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
  double Q[12] = {100, 200, 500,  500, 500, 300,  1, 1, 5,  8, 8, 1};

  // double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   2e-2, 2e-2, 2e-2, 2e-2, 2e-2, 2e-2}; // original hardware
  double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

  double *weights = Q;
  double *Alpha_K = Alpha;

  updateReferenceTrajectory(seResult, *stateCommand);

  // Timer t1;
  // t1.start();
  dtMPC = dt * iterationsBetweenMPC;
  // setup_problem(dtMPC, horizonLength, 0.25, 500);
  setup_problem(
  dtMPC, horizonLength, data._biped->mu, data._biped->f_max, data._biped->mass, 
  data._biped->I_body, data._biped->rl_params.A_residual, data._biped->rl_params.B_residual);
  
  // Timer t2;
  // t2.start();
  update_problem_data(p, v, q, w, r, yaw, weights, trajAll, Alpha_K, mpcTable, data);
  // std::cout << "\nMPC problem update took: " << t2.getMs() << " ms" << std::endl;

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
}


void ConvexMPCLocomotion::updateReferenceTrajectory(StateEstimate &seResult, DesiredStateCommand &stateCommand){
  // planar condition
  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = stateCommand.data.stateDes[2];
  yaw_desired += dt * turn_rate_des;

  stateCommand.data.stateDes[0] = world_position_desired[0];
  stateCommand.data.stateDes[1] = world_position_desired[1];
  stateCommand.data.stateDes[5] = yaw_desired;

  // if current position deviates from reference too much, reset reference close to current pose
  const double max_pos_error = 0.1;
  const double max_yaw_error = 0.3;
  double xStart = world_position_desired[0];
  double yStart = world_position_desired[1];
  double yawStart = yaw_desired;
  if(xStart - seResult.position[0] > max_pos_error) xStart = seResult.position[0] + max_pos_error;
  if(seResult.position[0] - xStart > max_pos_error) xStart = seResult.position[0] - max_pos_error;
  if(yStart - seResult.position[1] > max_pos_error) yStart = seResult.position[1] + max_pos_error;
  if(seResult.position[1] - yStart > max_pos_error) yStart = seResult.position[1] - max_pos_error;
  if(yawStart - seResult.rpy[2] > max_yaw_error) yawStart = seResult.rpy[2] + max_yaw_error;
  if(seResult.rpy[2] - yawStart > max_yaw_error) yawStart = seResult.rpy[2] - max_yaw_error;

  double trajInitial[12] = {stateCommand.data.stateDes[3],  // roll
                            stateCommand.data.stateDes[4],   // pitch
                            seResult.rpy[2], // yaw
                            // yawStart, // yaw
                            xStart, // x
                            yStart, // y
                            world_position_desired[2], // z
                            0, // wx
                            0, // wy
                            turn_rate_des,  // wz
                            v_des_world[0], // vx
                            v_des_world[1], // vy
                            0};   // vz

  // get trajectory though mpc horizon
  for (int i = 0; i < horizonLength; i++)
  {
    for (int j = 0; j < 12; j++)
      trajAll[12 * i + j] = trajInitial[j];

    if (v_des_world[0] < 0.01 && v_des_world[0] > -0.01) {
      trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
      }
    else{
      trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
    }

    if (v_des_world[1] < 0.01 && v_des_world[1] > -0.01) {
    trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
    }
    else{
      trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
    }

    if (turn_rate_des == 0){
    trajAll[12*i + 2] = trajInitial[2];
      }
    else{
    trajAll[12*i + 2] = seResult.rpy[2] + i * dtMPC * turn_rate_des;
    }
  }
}