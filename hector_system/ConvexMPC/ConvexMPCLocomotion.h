#ifndef CONVEXMPCLOCOMOTION_H
#define CONVEXMPCLOCOMOTION_H

#include "../include/common/FootSwingTrajectory.h"
#include "../include/common/SwingLegController.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"
#include "GaitGenerator.h"
#include <fstream>

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;

struct CMPC_Result {
  LegControllerCommand commands[2];
  Vec2<float> contactPhase;
};


class ConvexMPCLocomotion {
public:
  // ConvexMPCLocomotion(double _dt, int _iterations_between_mpc);
  ConvexMPCLocomotion(
      double _dt, 
      int _iterations_between_mpc, 
      int _horizon_length, 
      int _mpc_decimation);
  void reset(){
    firstRun = true;
    iterationCounter = 0;
  };

  void run(ControlFSMData& data);
  void setGaitNum(int gaitNum){gaitNumber = gaitNum % 2; if(gaitNum%2 ==0) gaitNumber = 2; return;}
  Vec3<double> pBody_des;
  Vec3<double> vBody_des;
  Vec3<double> aBody_des;

  Vec3<double> pBody_RPY_des;
  Vec3<double> vBody_Ori_des;

  Vec3<double> pFoot_des[2];
  Vec3<double> vFoot_des[2];
  Vec3<double> aFoot_des[2];

  Vec3<double> Fr_des[2];

  Vec2<double> contact_state;

  bool climb = 0;
  bool firstRun = true;
  // ofstream foot_position;

private:
  void updateMPC(int* mpcTable, ControlFSMData& data, bool omniMode);
  void updateReferenceTrajectory(StateEstimate &seResult, DesiredStateCommand &stateCommand);
  void updateGait(Vec2<int> dsp_durations, Vec2<int> ssp_durations);
  swingLegController swing;
  int iterationsBetweenMPC;
  int horizonLength;
  int mpc_decimation;
  double dt;
  double dtMPC;
  int iterationCounter = 0;
  Vec6<double> f_ff[2];
  Vec12<double> Forces_Sol;
  Vec2<double> swingTimes;
  FootSwingTrajectory<double> footSwingTrajectories[2];
  Gait walking, standing;
  Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
  bool firstSwing[2];
  double swingTimeRemaining[2];
  double stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<double> v_des_robot = {0, 0, 0};
  Vec3<double> v_des_world = {0, 0, 0};
  double turn_rate_des = 0; 
  Vec3<double> world_position_desired;
  double yaw_desired;
  Vec3<double> rpy_int;
  Vec3<double> rpy_comp;
  Vec3<double> pFoot[2];
  CMPC_Result result;
  double trajAll[12*36];

  Mat43<double> W; // W = [1, px, py]
  Vec3<double> a; // a = [a_0, a_1, a_2]; z(x,y) = a_0 + a_1x + a_2y
  Vec4<double> pz;
  double ground_pitch;
};


#endif //CONVEXMPCLOCOMOTION_H