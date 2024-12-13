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
                                                                                    walking(horizonLength, Vec2<int>(200, 200), Vec2<int>(0, 0)),
                                                                                    standing(horizonLength, Vec2<int>(int(0.0/_dt), int(0.0/_dt)), Vec2<int>(int(0.2/_dt), int(0.2/_dt)))
{
  dtMPC = dt * iterationsBetweenMPC;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;
    std::cout <<"Constructor"<<std::endl;
  // foot_position.open("foot_pos.txt");
  mpc_input.open("mpc_Input.txt");
  mpc_decimation = 5; // 5 control iterations per mpc (i.e. 200Hz)
}

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;

  //  auto* debugSphere = data.visualizationData->addSphere();
  //  debugSphere->color = {1,1,1,0.5};
  //  debugSphere->radius = 1;
  // data._legController->updateData();
  // data._stateEstimator->run();
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;
  // std::cout << "in side mpc" << seResult.rBody << std::endl;;
  // std::cout << "T265 Reading: " << std::endl;
  // std::cout << "x: " << seResult.position(0) << std::endl;
  // std::cout << "y: " << seResult.position(1) << std::endl;
  // std::cout << "z: " << seResult.position(2) << std::endl;



  // pick gait
  Gait *gait = &standing;
  if (gaitNumber == 1){
    gait = &standing; 
  }
  else if (gaitNumber==2){
    gait = &walking;
  }

  current_gait = gaitNumber;


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

    v_des_robot << 0, 0, 0; 
    v_des_world << 0, 0, 0;
    v_yaw_des = 0;

    for (int i = 0; i < 2; i++) // walking
    {
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  // ******* update reference trajectory ************
  v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;
  v_des_world = seResult.rBody.transpose() * v_des_robot;
  v_yaw_des = stateCommand->data.stateDes[11];

  if (gaitNumber == 1) // standing
  {
    // When standing, force velocity to 0
    v_des_robot << 0, 0, 0;
    v_des_world << 0, 0, 0; 
    v_yaw_des = 0;
  }

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = data._biped->ref_height;
  yaw_desired += dt * v_yaw_des;
  // ******* end of update reference trajectory ************


  // ************ foot placement planning ************
  swingTimes[0] = dt * gait->_swing(0);
  swingTimes[1] = dt * gait->_swing(1);

  double side_sign[2] = {1, -1};
  double interleave_y[2] = {-0.1, 0.1};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for (int i = 0; i < 2; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(data._biped->foot_height);
    Vec3<double> offset(0.0, side_sign[i] * 0.0, 0.0);
    Vec3<double> pRobotFrame = (data._biped->getHip2Location(i) + offset);
    Vec3<double> Pf = seResult.position +
                      seResult.rBody.transpose() * pRobotFrame 
                      + seResult.vWorld * swingTimeRemaining[i];

    double p_rel_max = 0.3;
    double k_x = 0.05; //kd for vx
    double k_y = 0.05; // kd for vy
    double pfx_rel = seResult.vWorld[0] * 0.5 * gait->_swing(i) * dt + k_x * (seResult.vWorld[0] - v_des_world[0]);
    double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_swing(i) * dt + k_y * (seResult.vWorld[1] - v_des_world[1]);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
    Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }
  // ************ end of foot placement planning ************

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

  // **** run MPC ****
  int *mpcTable = gait->mpc_gait(iterationsBetweenMPC);
  if ((iterationCounter % mpc_decimation) == 0){
    updateMPC(mpcTable, data, omniMode);
  }
  // **** end of MPC ****

  // ************ swing foot trajectory planning and legcontroller command set ************
  //  StateEstimator* se = hw_i->state_estimator;
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

      double side = (foot == 1) ? 1.0 : -1.0; //Left foot (0) side = -1.0, Right foot (1) side = 1.0
      Vec3<double> dummyPos = {seResult.position[0],seResult.position[1],seResult.position[2]};
      Vec3<double> dummyVel = {seResult.vWorld[0],seResult.vWorld[1],seResult.vWorld[2]};
      Vec3<double> hipOffset = {-0.025, side*-0.06, 0};
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - dummyPos) + hipOffset;
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - dummyVel);
      // Vec3<double> vDesLeg = seResult.rBody * (0*vDesFootWorld - dummyVel); // touch down with zero velocity

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
      data._legController->commands[foot].kptoe = 10; // 0
      data._legController->commands[foot].kdtoe = 0.2;
      se_contactState[foot] = contactState;
    }

    else if (contactState > 0) // foot is in stance
    { 
      firstSwing[foot] = true;
      Vec3<double> pDesLeg = {0, 0, 0};
      Vec3<double> vDesLeg = {0, 0, 0};
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp_stance; 
      data._legController->commands[foot].kdCartesian = Kd_stance;

      data._legController->commands[foot].kptoe = 0; 
      data._legController->commands[foot].kdtoe = 0;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];
      se_contactState[foot] = contactState;
    }

    data._stateEstimator->setContactPhase(se_contactState);
  }
  // ************ end of swing foot trajectory planning and legcontroller command set ************

  // tick control counter
  iterationCounter++;
}

void ConvexMPCLocomotion::updateMPC(int *mpcTable, ControlFSMData &data, bool omniMode)
{
  auto seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  double *p = seResult.position.data();
  double *v = seResult.vWorld.data();
  double *w = seResult.omegaWorld.data();
  double *q = seResult.orientation.data();

  // float* pf = (float*) seResult.position.data();
  // float* vf = (float*) seResult.vWorld.data();
  // float* wf = (float*) seResult.omegaWorld.data();
  // float* qf = (float*) seResult.orientation.data();

  double r[6];
  for (int i = 0; i < 6; i++)
  {
    r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
  }

  // double Q[12] = {200, 150, 10,   430, 430, 460,   1, 1, 1,   1, 1, 3}; // state weight (original -> p_z weight is too low??)
  double Q[12] = {200, 150, 300,  300, 200, 300,  1, 1, 2.0,  2.0, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz (from software code)
  // double Q[12] = {150, 50, 100,  700, 250, 350,  .5, .5, .5,  .5, .5, .5}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
  //  double Q[12] = {1000, 1000, 1000,   1000, 1000, 5000,   1, 1, 1,   1, 1, 1};
  // double Q[12] = {3.5*1000, 3.5*1000, 1000,   1000, 1000, 0.25*10000,   1000, 2*1000, 1000,   1000, 1000, 3*1000};
  //double Q[12] = {2000, 2000, 10, 35, 35, 35, 1, 1, 1, 1, 1, 1};

  // double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   2e-2, 2e-2, 2e-2, 2e-2, 2e-2, 2e-2}; // control weight (original)
  double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2}; // control weight (from software code)
  // double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   5e-4, 5e-4, 5e-4, 5e-4, 5e-4, 5e-4};

  double *weights = Q;
  double *Alpha_K = Alpha;

  double yaw = seResult.rpy[2];
  const double max_pos_error = 0.1;
  const double max_yaw_error = 0.3;
  double xStart = world_position_desired[0];
  double yStart = world_position_desired[1];
  double yawStart = yaw_desired;


  if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
  if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

  if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
  if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

  if(yaw_desired - yaw > max_yaw_error) yaw_desired = yaw + max_yaw_error;
  if(yaw - yaw_desired > max_yaw_error) yaw_desired = yaw - max_yaw_error;

  world_position_desired[0] = xStart;
  world_position_desired[1] = yStart;
  yaw_desired = yawStart;

  double roll_comp = 0.0;
  double pitch_comp = 0.0;
  Vec3<double> foot_center = (pFoot[0]+pFoot[1])/2.0;
  std::cout << "pFoot 0: " << pFoot[0] << std::endl;
  std::cout << "pFoot 1: " << pFoot[1] << std::endl;
  
  if (gaitNumber == 1)
  { //standing
    roll_comp = v_des_robot[0]*0.5;
    pitch_comp = v_des_robot[1]*0.5;
  }

  else 
  {
    // walking
    roll_comp = -v_yaw_des/30.0;
    pitch_comp = -v_des_robot[0]*0.15;
  }


  double trajInitial[12] = {roll_comp,  // roll
                            pitch_comp,    // pitch
                            yaw,    // yaw
                            xStart, // x
                            yStart,  // y
                            data._biped->ref_height, // z
                            0, // wx
                            0, // wy
                            v_yaw_des,  // wz
                            v_des_world[0], // vx
                            v_des_world[1], // vy
                            0}; // vz
  

  for (int i = 0; i < horizonLength; i++)
  {
    for (int j = 0; j < 12; j++)
      trajAll[12 * i + j] = trajInitial[j];
    
    // compute reference x position during horizon
    if (v_des_world[0] < 0.01 && v_des_world[0] > -0.01)
    {
      trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
    }
    else
    {
        trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
    }

    // compute reference y position during horizon
    if (v_des_world[1] < 0.01 && v_des_world[1] > -0.01)
    {
      trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
    }
    else
    {
        trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
    }

    // compute reference yaw angle during horizon
    if (v_yaw_des == 0)
    {
      trajAll[12*i + 2] = trajInitial[2];
    }
    else
    {
      trajAll[12*i + 2] = yaw + i * dtMPC * v_yaw_des;
    }

    std::cout << "traj " << i << std::endl;
    for (int j = 0; j < 12; j++) {
      std::cout << trajAll[12 * i + j] << "  ";
    }
  }

  Timer t1;
  t1.start();
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.25, 500);
  Timer t2;
  t2.start();
  // cout << "dtMPC: " << dtMPC << "\n";
  //  update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
  update_problem_data(p, v, q, w, r, yaw, weights, trajAll, Alpha_K, mpcTable, data);

  for (int i = 0; i < 3; i++){
    mpc_input << p[i] << "  ";
  }
  for (int i = 0; i < 3; i++){
    mpc_input << v[i] << "  ";
  }
  for (int i = 0; i < 4; i++){
    mpc_input << q[i] << "  ";
  }
  for (int i = 0; i < 3; i++){
    mpc_input << w[i] << "  ";
  }
  for (int i = 0; i < 6; i++){
    mpc_input << r[i] << "  ";
  }
  mpc_input << std::endl;
  
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

    for (int i = 0; i < 3; i++)
    {
      f(i) = GRF_R(i);
      f(i+3) = GRM_R(i);
    }
    f_ff[leg] = f;

    // std::cout << f_ff[leg] << std::endl;
    std::cout << "mpc solution" << leg << "\n" << f << std::endl;
    //  Update for WBC
    //  Fr_des[leg] = f;
  }
  contact_state(0) = mpcTable[0];
  contact_state(1) = mpcTable[1];
}