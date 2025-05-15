#pragma once

#include <memory>
#include <iostream>
#include "../include/common/cppTypes.h"

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;
using namespace std;

/**
 * @file gaitgenerator.h
 * @brief Gait Generation for Bipedal Locomotion
 *
 * This file provides the definition for the Gait class, which is responsible for
 * generating and managing various gait patterns for a bipedal robot.
 * This is the updated version of GaitGenerator.cpp where assymetric gait and double support phase are added. 

 *
 */


class Gait
{
public:
  Gait(int mpc_horizon, Vec2<int> dsp_durations, Vec2<int> ssp_durations, double dt, double dt_mpc);
  ~Gait();
  void update_parameter(Vec2<int> dsp_durations, Vec2<int> ssp_durations);
  Vec2<double> getContactSubPhase();
  Vec2<double> getSwingSubPhase();
  int* mpc_gait();
  void updatePhase();
  void updateSamplingTime(double dt_mpc);
  void reset(){_gait_phase = 0;};
  Vec2<int> _stance; 
  Vec2<int> _swing;
  Vec2<double> _swing_durations_sec; 
  Vec2<double> _stance_durations_sec;
  double gait_durations_sec;


private:
  int _mpc_horizon;
  // std::shared_ptr<int[]> _mpc_table; // NOT supported in CXX14
  std::unique_ptr<int[]> _mpc_table;
  
  int _gait_time_step = 0;
  double _gait_phase = 0;
  double _dt;
  double _dt_mpc; 
  double _stepping_frequency = 1.0;

  Array2i _dsp_durations;                   // duration of double support phase in mpc segments
  Array2d _dsp_durations_phase;          // duration of double support phase in gait phase
  Array2i _ssp_durations;              // duration of single support phase in mpc segments
  Array2d _ssp_durations_phase;        // duration of single support phase in gait phase
  int _gait_cycle_length;


  Array2i _stance_durations;           // durations in mpc segments
  Array2d _stance_durations_phase;    // durations in phase (0 to 1)

  Array2i _swing_durations;           // durations in mpc segments
  Array2d _swing_durations_phase;    // durations in phase (0 to 1)

};