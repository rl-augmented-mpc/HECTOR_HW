#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include "../include/common/ControlFSMData.h"


struct problem_setup
{
  float dt;
  float mu;
  float f_max;
  int horizon;
};

struct update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[12];
  float yaw;
  float weights[12];
  float traj[12*K_MAX_GAIT_SEGMENTS];
  
  // float alpha;
  float Alpha_K[12];
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
// EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
//                                         float* r, float yaw, float* weights,
//                                         float* state_trajectory, float alpha, int* gait);
// EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);

EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double* Alpha_K, int* gait, ControlFSMData& data);


EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float* Alpha_K, int* gait, ControlFSMData& data);
#endif