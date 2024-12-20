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

  current_gait = gaitNumber;
  Vec3<double> v_des_robot;
  Vec3<double> v_des_world;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.55; //.5;;;
  yaw_desired += dt * stateCommand->data.stateDes[11];

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() 
    * (data._biped->getHip2Location(i) + data._legController->data[i].p);
  }

  // some first time initialization
  if (firstRun)
  {
    // std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0, 0, 0); // connect to desired state command later
    Vec3<double> v_des_world(0, 0, 0); // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    //
    if (gaitNumber == 7)
    {
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];
      pBody_des[2] = 0.55;

      vBody_des[0] = 0;
      vBody_des[0] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }


  // foot placement
  swingTimes[0] = dt * gait->_swing[0];
  swingTimes[1] = dt * gait->_swing[1];

  double side_sign[2] = {1, -1};
  double interleave_y[2] = {-0.1, 0.1};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for (int i = 0; i < 2; i++)
  {
    // update swing time
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    // *********** foot placement planning ***********
    double foot_height = 0.12;
    footSwingTrajectories[i].setHeight(foot_height);
    Vec3<double> offset(0.0, side_sign[i] * 0.0, 0.0);
    Vec3<double> pRobotFrame = (data._biped->getHip2Location(i) + offset);
 
    Vec3<double> Pf = seResult.position + seResult.rBody.transpose() * pRobotFrame + seResult.vWorld * swingTimeRemaining[i];
    double p_rel_max = 0.3;
    double k_x = 0.05; 
    double k_y = 0.05;
    double pfx_rel = seResult.vWorld[0] * 0.5 * gait->_swing(i) * dt + k_x * (seResult.vWorld[0] - v_des_world[0]);
    double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_swing(i) * dt + k_y * (seResult.vWorld[1] - v_des_world[1]);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  // update gait phase
  float stepping_frequency = 1.0; 
  gait->updatePhase(stepping_frequency);

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
  
  
  updateMPCIfNeeded(mpcTable, data, omniMode);
  iterationCounter++;

  Vec2<double> se_contactState(0, 0);

  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot); 
    Vec3<double> pFootWorld;

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        pFootWorld = pFoot[foot];
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      double side = -1.0 ;
      if (foot == 1){
        side = -1.0;}
    else if (foot == 0){
        side = 1.0;
    }
      Vec3<double> dummyPos = {seResult.position[0],seResult.position[1],seResult.position[2]};
      Vec3<double> dummyVel = {seResult.vWorld[0],seResult.vWorld[1],seResult.vWorld[2]};
      Vec3<double> hipOffset = {0.025, side*-0.06, -0.136*0};
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - dummyPos) - hipOffset;
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - dummyVel);
      if (vDesLeg.hasNaN())
      {
        vDesLeg << 0, 0, 0;
      }

      if (pDesLeg.hasNaN())
      {
        pDesLeg << 0, 0, -0.4;
      }

      data._legController->commands[foot].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      // std::cout << "check 3" << std::endl;
      data._legController->commands[foot].kptoe = 10; // 0
      data._legController->commands[foot].kdtoe = 0.2;
      data._legController->commands[foot].control_mode = int(ControlMode::SWING);
      se_contactState[foot] = contactState;
    }

    else if (contactState > 0) // foot is in stance
    { 
      firstSwing[foot] = true;
      Vec3<double> pDesLeg = {0, 0, 0};
      Vec3<double> vDesLeg = {0, 0, 0};
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp_stance; // 0
      data._legController->commands[foot].kdCartesian = Kd_stance;

      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];

      se_contactState[foot] = contactState;
      data._legController->commands[foot].tau = data._legController->data[foot].J.transpose()* f_ff[foot];
      data._legController->commands[foot].control_mode = int(ControlMode::STANCE);
    }

    // se->set_contact_state(se_contactState); todo removed
    data._stateEstimator->setContactPhase(se_contactState);
    // data._legController->updateCommand();
  }
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode)
{
  if ((iterationCounter % 5) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *q = seResult.orientation.data();

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }

    // double Q[12] = {200, 150, 10,   430, 430, 460,   1, 1, 1,   1, 1, 3};
    double Q[12] = {300, 300, 150,   300, 300, 100,   1, 1, 1,   5, 3, 3};

    double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   2e-2, 2e-2, 2e-2, 2e-2, 2e-2, 2e-2};
    // double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   5e-4, 5e-4, 5e-4, 5e-4, 5e-4, 5e-4};

    double *weights = Q;
    double *Alpha_K = Alpha;
 
    double yaw = seResult.rpy[2];
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0);

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    const double max_pos_error = 0.1;
    const double max_yaw_error = 0.3;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];
    double yawStart = yaw_desired;

    double yaw_des = v_des_robot[1] * 6;
    double height_add_des = v_des_robot[0] * 0.5;

    if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
    if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

    if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
    if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

    if(yaw_desired - yaw > max_yaw_error) yaw_desired = yaw + max_yaw_error;
    if(yaw - yaw_desired > max_yaw_error) yaw_desired = yaw - max_yaw_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;
    yaw_desired = yawStart;

    double yaw_rate_des = 0.0;
    double roll_comp = 0.0;
    double pitch_comp = 0.0;
    Vec3<double> foot_center = (pFoot[0]+pFoot[1])/2.0;
    
    if (gaitNumber == 7) { //standing
      v_des_world[0] = 0;
      v_des_world[1] = 0;
      roll_comp = v_des_robot[0]*0.5;
      pitch_comp = v_des_robot[1]*0.5;
    }
    else {
      height_add_des = 0;
      yaw_rate_des = v_des_world[1] * 5.0;
      roll_comp = -stateCommand->data.stateDes[11]/30.0;
      pitch_comp = -v_des_robot[0]*0.15;
    }

    double trajInitial[12] = {roll_comp,  // 0
                              pitch_comp,    // 1
                              seResult.rpy[2],    // 2
                              xStart*1.0,                                   // 3
                              yStart*1.0,                                   // 4
                              0.55, // 5
                              0,                                        // 6
                              0,                                        // 7
                              stateCommand->data.stateDes[11],  // 8
                              v_des_world[0],                           // 9
                              v_des_world[1],                           // 10
                              0};   // 11
    

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
        if (stateCommand->data.stateDes[11] == 0){
        trajAll[12*i + 2] = trajInitial[2];
         }
        else{
        trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
        }
    }

    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 0.25, 500);
    Timer t2;
    t2.start();
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
}