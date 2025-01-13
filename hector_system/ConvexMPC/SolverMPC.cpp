
#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <cmath>
#include "../third_party/qpOASES/include/qpOASES.hpp"
#include <stdio.h>
#include <sys/time.h>
#include "../include/common/Utilities/Timer.h"
#include <fstream>

// #define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
// big enough to act like infinity, small enough to avoid numerical weirdness.

RobotState rs;
using Eigen::Dynamic;
using std::cout;
using std::endl;
using std::ofstream;

// qpOASES::real_t a;

Matrix<fpt, Dynamic, 13> A_qp;
Matrix<fpt, Dynamic, Dynamic> B_qp;
Matrix<fpt, 13, 12> Bdt;
Matrix<fpt, 13, 13> Adt;
Matrix<fpt, 25, 25> ABc, expmm;
Matrix<fpt, Dynamic, Dynamic> S;
Matrix<fpt, Dynamic, 1> X_d;
Matrix<fpt, Dynamic, 1> U_b;
Matrix<fpt, Dynamic, 1> L_b;
Matrix<fpt, Dynamic, Dynamic> fmat;
Matrix<fpt, Dynamic, Dynamic> qH;
Matrix<fpt, Dynamic, 1> qg;
Matrix<fpt, 16, 12> F_control;
Matrix<fpt, 1, 3> tx_F;
Matrix<fpt, 1, 3> ty_F;
Matrix<fpt, 1, 3> DNFG;

// Matrix<fpt,Dynamic,Dynamic> eye_12h;
Matrix<fpt, Dynamic, Dynamic> Alpha_diag;
Matrix<fpt, Dynamic, Dynamic> Alpha_rep;

qpOASES::real_t *H_qpoases;
qpOASES::real_t *g_qpoases;
qpOASES::real_t *A_qpoases;
qpOASES::real_t *lb_qpoases;
qpOASES::real_t *ub_qpoases;
qpOASES::real_t *q_soln;

qpOASES::real_t *H_red;
qpOASES::real_t *g_red;
qpOASES::real_t *A_red;
qpOASES::real_t *lb_red;
qpOASES::real_t *ub_red;
qpOASES::real_t *q_red;
u8 real_allocated = 0;

char var_elim[2000];
char con_elim[2000];

Matrix<fpt, 3, 3> euler_to_rotation(fpt roll, fpt pitch, fpt yaw) {
    
    fpt r = roll;
    fpt p = pitch;
    fpt y = yaw;

    // Calculate rotation matrix
    Matrix<fpt, 3, 3> Rx, Ry, Rz, Rb;
    Rx << 1, 0, 0,
          0, cos(r), -sin(r),
          0, sin(r), cos(r);
    Ry << cos(p), 0, sin(p),
          0, 1, 0,
          -sin(p), 0, cos(p);
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;
    // Rb << 1, sin(r)*tan(p), cos(r)*tan(p),
    //       0, cos(r), -sin(r),
    //       0, sin(r)/cos(p), cos(r)/cos(p);  
    Rb << cos(y)*cos(p), -sin(y), 0,
          sin(y)*cos(p), cos(y), 0,
          -sin(p), 0, 1; 
    Matrix<fpt, 3, 3> R = Rb.inverse();
    // Eigen::Matrix3d R = Rx * Ry * Rz;   
    return R;
}

// Returns QP solution
mfp *get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.001 && a > -.001);
}

s8 near_one(fpt a)
{
  return near_zero(a - 2);
}

// Sets parameter matrices to qpOASES type:
void matrix_to_real(qpOASES::real_t *dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for (s16 r = 0; r < rows; r++)
  {
    for (s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r, c);
      a++;
    }
  }
}

void c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, fpt dt, s16 horizon)
{
  if (horizon > 19)
  {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt, 13, 13> Acd = Matrix<fpt, 13, 13>::Identity() + dt * Ac;
  Matrix<fpt, 13, 12> Bcd = dt * Bc;

  for (int i = 0; i < 10; i++)
  {
    Eigen::Matrix<fpt, 13, 13> Acdm;
    Acdm = Matrix<fpt, 13, 13>::Identity();
    for (int j = 0; j < i + 1; j++)
    {
      Acdm *= Acd;
    }

    A_qp.block<13, 13>(i * 13, 0) << Acdm;
  }

  Eigen::Matrix<fpt, 13, 13> Acdp;
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < i + 1; j++)
    {
      Acdp = Matrix<fpt, 13, 13>::Identity();
      for (int k = 0; k < i - j; k++)
      {
        Acdp *= Acd;
      }

      if (i - j == 0)
      {
        Acdp = Matrix<fpt, 13, 13>::Identity();
      }

      B_qp.block<13, 12>(i * 13, j * 12) << Acdp * Bcd;
    }
  }

  for (int i = 0; i < 10; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      B_qp.block<13, 12>(i * 13, j * 12) << Matrix<fpt, 13, 12>::Zero();
    }
  }
}

// Resizing & initaliztation:
void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon * horizon;

  A_qp.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon * 1;

  B_qp.resize(13 * horizon, 12 * horizon);
  mcount += 13 * h2 * 12;

  S.resize(13 * horizon, 13 * horizon);
  mcount += 13 * 13 * h2;

  X_d.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon;

  U_b.resize(16*horizon, Eigen::NoChange);
  mcount += 16*horizon;

  L_b.resize(16*horizon, Eigen::NoChange);
  mcount += 16*horizon;

  fmat.resize(16*horizon, 12*horizon);
  mcount += 16*12*h2;

  qH.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * h2;

  qg.resize(12 * horizon, Eigen::NoChange);
  mcount += 12 * horizon;

  // eye_12h.resize(12*horizon, 12*horizon);
  // mcount += 12*12*horizon;

  Alpha_rep.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * horizon;

  // printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  L_b.setZero();
  fmat.setZero();
  qH.setZero();
  // eye_12h.setIdentity();
  Alpha_rep.setZero();

  // TODO: use realloc instead of free/malloc on size changes

  if (real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*16*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*16*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(16*1*horizon*sizeof(qpOASES::real_t));
  mcount += 16*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(16*1*horizon*sizeof(qpOASES::real_t));
  mcount += 16*horizon;
  q_soln = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*16*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*16*h2;
  lb_red = (qpOASES::real_t*)malloc(16*1*horizon*sizeof(qpOASES::real_t));
  mcount += 16*horizon;
  ub_red = (qpOASES::real_t*)malloc(16*1*horizon*sizeof(qpOASES::real_t));
  mcount += 16*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;

  // printf("malloc'd %d floating point numbers.\n",mcount);

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

// Matrix Operations:
inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
  Matrix<fpt, 3, 3> cm;
  cm << 0.f, -r(2), r(1),
      r(2), 0.f, -r(0),
      -r(1), r(0), 0.f;
  return I_inv * cm;
}

// continuous time state space matrices with residual matrices
// https://arxiv.org/abs/2104.00065 eq.8-9
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt,13,13> A_residual, Matrix<fpt,13,12> B_residual, Matrix<fpt, 3, 2> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13> &A, Matrix<fpt, 13, 12> &B)
{
  A.setZero();
  A.block<3, 3>(0, 6) << R_yaw;
  A.block<3, 3>(3, 9) << Matrix<fpt, 3, 3>::Identity();
  A.block<3, 1>(9, 12) << 0, 0, -9.81f;
  A += A_residual;

  B.setZero();
  Matrix<fpt, 3, 3> I_inv = I_world.inverse();
  for (s16 b = 0; b < 2; b++)
  {
    B.block<3, 3>(6, b * 3) << cross_mat(I_inv, r_feet.col(b));
  }
  B.block<3, 3>(6, 6) << I_inv ;
  B.block<3, 3>(6, 9) << I_inv ;  
  B.block<3, 3>(9, 0) << Matrix<fpt, 3, 3>::Identity() / m ; 
  B.block<3, 3>(9, 3) << Matrix<fpt, 3, 3>::Identity() / m ;
  B += B_residual;
}

void quat_to_rpy(Quaternionf q, Matrix<fpt, 3, 1> &rpy)
{
  // from my MATLAB implementation

  // edge case!
  fpt as = t_min(2. * (q.w() * q.y() - q.x() * q.z()), .99999);
  rpy(0) = atan2(2.f * (q.w() * q.x() + q.y() * q.z()), 1. - 2.f * (sq(q.x()) + sq(q.y())));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f * (q.w() * q.z() + q.x() * q.y()), 1. - 2.f * (sq(q.y()) + sq(q.z())));
  // std::cout << "MPC solver rpy: " << rpy(0) << " " << rpy(1) << " " << rpy(2) << std::endl;
}
void print_problem_setup(problem_setup *setup)
{
  printf("DT: %.3f\n", setup->dt);
  printf("Mu: %.3f\n", setup->mu);
  printf("F_Max: %.3f\n", setup->f_max);
  printf("Horizon: %d\n", setup->horizon);
}

void print_update_data(update_data_t *update, s16 horizon)
{
  print_named_array("p", update->p, 1, 3);
  print_named_array("v", update->v, 1, 3);
  print_named_array("q", update->q, 1, 4);
  print_named_array("w", update->r, 3, 4);
  pnv("Yaw", update->yaw);
  print_named_array("weights", update->weights, 1, 12);
  print_named_array("trajectory", update->traj, horizon, 12);
  print_named_array("Alpha", update->Alpha_K, 1, 12);
  print_named_array("gait", update->gait, horizon, 4);
}

Matrix<fpt, 13, 1> x_0;
Matrix<fpt, 3, 3> I_world;
Matrix<fpt, 13, 13> A_ct;
Matrix<fpt, 13, 12> B_ct_r;

// Main function:
void solve_mpc(update_data_t *update, problem_setup *setup, ControlFSMData &data)
{

  Eigen::Matrix<double, 10, 1> q;
  // for (int i = 0; i < 2; i++)
  // {
  //   for (int k = 0; k < 5; k++)
  //   {
  //     q(i * 5 + k) = data._legController->data[i].q(k);
  //   }
  // }

  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);

  #ifdef K_PRINT_EVERYTHING

    printf("-----------------\n");
    printf("   PROBLEM DATA  \n");
    printf("-----------------\n");
    print_problem_setup(setup);

    printf("-----------------\n");
    printf("    ROBOT DATA   \n");
    printf("-----------------\n");
    rs.print();
    print_update_data(update, setup->horizon);
  #endif

  // roll pitch yaw
  Matrix<fpt, 3, 1> rpy;
  quat_to_rpy(rs.q, rpy);
  // Eigen::Matrix3d rot_mat;
  Matrix<fpt, 3, 3> Rb;
  Rb = euler_to_rotation(rpy(0), rpy(1), rpy(2));
  
  x_0 << rpy(0), rpy(1), rpy(2), rs.p, rs.w, rs.v, 1.f;
  I_world = rs.R * setup->I_body * rs.R.transpose();
  ct_ss_mats(I_world, setup->mass, setup->A_residual, setup->B_residual, rs.r_feet, Rb, A_ct, B_ct_r);
 
  // Rotation of Foot:
  Matrix<fpt, 3, 3> R_foot_L;
  Matrix<fpt, 3, 3> R_foot_R;
  R_foot_L << 1,0,0,
              0,1,0,
              0,0,1;
  R_foot_R << 1,0,0,
              0,1,0,
              0,0,1;
  // R_foot_L << - 1.0*sin(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - cos(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))), -1.0*cos(q(1))*sin(q(0)), cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - sin(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))),
  //             cos(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) - 1.0*sin(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),      cos(q(0))*cos(q(1)), cos(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) + sin(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),
  //                                                                                                                                                                                                                           -1.0*sin(q(2) + q(3) + q(4))*cos(q(1)),              sin(q(1)),                                                                                                                                                                                                                             cos(q(2) + q(3) + q(4))*cos(q(1));
  // R_foot_R << - 1.0*sin(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - cos(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))), -1.0*cos(q(6))*sin(q(5)), cos(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - sin(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))),
  //             cos(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) - 1.0*sin(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),      cos(q(5))*cos(q(6)), cos(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) + sin(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),
                                                                                                                                                                                                                            // -1.0*sin(q(7) + q(8) + q(9))*cos(q(6)),              sin(q(6)),                                                                                                                                                                                                                             cos(q(7) + q(8) + q(9))*cos(q(6));

  c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);

  // weights
  Matrix<fpt, 13, 1> full_weight;
  for (u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon, 1);

  // reference trajectory
  for (s16 i = 0; i < setup->horizon; i++)
  {
    for (s16 j = 0; j < 12; j++)
      X_d(13 * i + j, 0) = update->traj[12 * i + j];
    X_d(13 * i + 12, 0) = 1.f;
  }

  // QP Upper Bound
  // QP Lower Bound is set to 0 by default
  for(s16 i = 0; i < setup->horizon; i++){
      U_b(0 + 16*i) = BIG_NUMBER;
      U_b(1 + 16*i) = BIG_NUMBER;
      U_b(2 + 16*i) = BIG_NUMBER;
      U_b(3 + 16*i) = BIG_NUMBER;

      U_b(4 + 16*i) = 0.01;
      U_b(5 + 16*i) = BIG_NUMBER;      
      U_b(6 + 16*i) = BIG_NUMBER;
      U_b(7 + 16*i) = setup->f_max * update->gait[2*i + 0] ;

      U_b(8 + 16*i) = BIG_NUMBER;
      U_b(9 + 16*i) = BIG_NUMBER;
      U_b(10 + 16*i) = BIG_NUMBER;
      U_b(11 + 16*i) = BIG_NUMBER;
      
      U_b(12 + 16*i) = 0.01;
      U_b(13 + 16*i) = BIG_NUMBER;      
      U_b(14 + 16*i) = BIG_NUMBER;
      U_b(15 + 16*i) = setup->f_max * update->gait[2*i + 1];
      }


  // Initalization of Line Contact Constraint Parameters
  fpt mu = 1.f/setup->mu;
  fpt lt = 0.07; // length of toe from ankle link
  fpt lh = 0.04; // length of heel from ankle link

  // Matrix<fpt,5,3> f_block;
  Matrix<fpt,10,12> f_blockz;
  Matrix<fpt,16,12> F_control;

  Matrix<fpt,1,3> lt_vec;
  Matrix<fpt,1,3> lt_3D;
  lt_vec << 0, 0, lt;

  Matrix<fpt,1,3> lh_vec;
  Matrix<fpt,1,3> lh_3D;
  lh_vec << 0, 0, lh;

  Matrix<fpt,1,3> M_vec;
  M_vec << 0, 1.00, 0;
  Matrix<fpt,1,3> M_3D;

  Matrix<fpt,1,3> Moment_selection(1.f, 0, 0);
  Matrix<fpt,1,3> Moment_selection_3D;

  // **** Input constraints ****
  // https://arxiv.org/abs/2312.11868 eq13b-e
  // control input is [F1, F2, M1, M2]
  
  // Friction pyramid (-mu*Fz < Fx < mu*Fz, -mu*Fz < Fy < mu*Fz)
  // 0 < -1/mu * Fx + Fz
  // 0 < 1/mu * Fx + Fz
  // 0 < -1/mu * Fy + Fz
  // 0 < 1/mu * Fy + Fz

  // X moment zero (Mx = 0)
  // moment_selection * R_foot (wrt world frame) * M = 0

  // Line contact (lt*Fz < Mx < -lt*Fz, -lh*Fz < My < lh*Fz)
  // 0 < [0, 0, lt] * R_foot * F + [0, 1, 0] * R_foot.T * M
  // 0 < [0, 0, lh] * R_foot * F - [0, 1, 0] * R_foot.T * M

  // Fz constraints
  // 0 < Fz < maxGRF

  F_control.setZero();

  //leg 1
  F_control.block<1, 12>(0, 0) //Friction leg 1
      << -mu, 0, 1.f,   0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(1, 0)
      <<  mu, 0, 1.f,   0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(2, 0)
      <<  0, -mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(3, 0)
      <<  0,  mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;

  F_control.block<1, 12>(4, 0) //Mx Leg 1
      << 0, 0, 0, 0, 0, 0, Moment_selection * R_foot_L.transpose()* rs.R.transpose(), 0, 0, 0;

  F_control.block<1, 12>(5, 0) //Line Leg 1
      << lt_vec * R_foot_L.transpose()* rs.R.transpose(),   0, 0, 0,    M_vec * R_foot_L.transpose()* rs.R.transpose(),   0, 0, 0;
  F_control.block<1, 12>(6, 0)
      << lh_vec * R_foot_L.transpose()* rs.R.transpose(),   0, 0, 0,   -M_vec * R_foot_L.transpose()* rs.R.transpose(),   0, 0, 0;
      
  F_control.block<1, 12>(7, 0) //Fz Leg 1
      << 0, 0, 2.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  //leg 2
  F_control.block<1, 12>(8, 0) //Friction leg 2
      <<  0, 0, 0,   -mu, 0, 1.f,   0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(9, 0)
      <<  0, 0, 0,    mu, 0, 1.f,   0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(10, 0)
      <<   0, 0, 0,   0, -mu, 1.f,  0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(11, 0)
      <<   0, 0, 0,   0, mu, 1.f,   0, 0, 0, 0, 0, 0;     

  F_control.block<1, 12>(12, 0) //Mx Leg 1
      << 0, 0, 0, 0, 0, 0, 0, 0, 0, Moment_selection * R_foot_R.transpose()* rs.R.transpose();

  F_control.block<1, 12>(13, 0) //Line Leg 2
      << 0, 0, 0,   lt_vec * R_foot_R.transpose()* rs.R.transpose(),   0, 0, 0,    M_vec * R_foot_R.transpose()* rs.R.transpose();
  F_control.block<1, 12>(14, 0)
      << 0, 0, 0,   lh_vec * R_foot_R.transpose()* rs.R.transpose(),   0, 0, 0,   -M_vec * R_foot_R.transpose()* rs.R.transpose();  
  
  F_control.block<1, 12>(15, 0)  //Fz Leg 2
      << 0, 0, 0, 0, 0, 2.f, 0, 0, 0, 0, 0, 0;

  // Set to fmat QP
  for(s16 i = 0; i < setup->horizon; i++)
  {
    fmat.block(i*16,i*12,16,12) = F_control;
  }
  // Construct K:
  Alpha_diag.resize(12, 12);
  Alpha_diag.setZero();

  for (s16 i = 0; i < 12; i++)
  {
    Alpha_diag.block(i, i, 1, 1) << update->Alpha_K[i];
  }
  for (s16 i = 0; i < setup->horizon; i++)
  {
    Alpha_rep.block(i * 12, i * 12, 12, 12) << Alpha_diag;
  }
  // Equivalent to Matlab Formulaion
  Timer timer;
  timer.start();
  qH = 2 * (B_qp.transpose() * S * B_qp + Alpha_rep);
  qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);
  // std::cout << "\nTime to construct QP: " << timer.getMs() << " ms" << std::endl;
  

  // Calls function that sets parameters matrices in qpOASES types
  matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);
  matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);
  matrix_to_real(A_qpoases,fmat,setup->horizon*16, setup->horizon*12);
  matrix_to_real(ub_qpoases,U_b,setup->horizon*16, 1);
  // matrix_to_real(lb_qpoases,L_b,setup->horizon*16, 1);
  for(s16 i = 0; i < 16*setup->horizon; i++){
    lb_qpoases[i] = 0.0f;
  }

  s16 num_constraints = 16*setup->horizon;
  s16 num_variables = 12*setup->horizon;

  // Max # of working set recalculations
  qpOASES::int_t nWSR = 200;

  int new_vars = num_variables;
  int new_cons = num_constraints;

  for (int i = 0; i < num_constraints; i++)
    con_elim[i] = 0;

  for (int i = 0; i < num_variables; i++)
    var_elim[i] = 0;

  for (int i = 0; i < num_constraints; i++)
  {
    if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
      // printf("check 1\n");
      // std::cout<< i << std::endl;
      
    double *c_row = &A_qpoases[i * num_variables];
    // std::cout << "c_row: " << std::endl;
    for (int j = 0; j < num_variables; j++)
    {
      
      // std::cout << c_row[j] << std::endl;
      if (near_one(c_row[j]))
      {
          new_vars -= 6;
          new_cons -= 8;
          // // int cs = ((j)*13)/6 + 6;//j+2*i;//(j*7)/6 -6;
          // int cs = ceil((j+4)/6)*13-1;
          int cs;
          if (j%2 == 0){
            cs = (j+4)/6*8-1;
          }
          else{
            cs = (j+1)/6*8+7;
          }

          // std::cout << "j = " << j << "; " << "cs: " << cs <<std::endl;
          // var_elim[j-8] = 1;
          // var_elim[j-7] = 1;
          // var_elim[j-6] = 1;
          var_elim[j+6] = 1;
          var_elim[j+5] = 1;
          var_elim[j+4] = 1;
          var_elim[j-2] = 1;
          var_elim[j-1] = 1;
          var_elim[j  ] = 1;
          
          con_elim[cs-0] = 1;
          con_elim[cs-1] = 1;
          con_elim[cs-2] = 1;
          con_elim[cs-3] = 1;
          con_elim[cs-4] = 1;         
          con_elim[cs-5] = 1;
          con_elim[cs-6] = 1;
          con_elim[cs-7] = 1;
          
      }
    }
  }
  
  if (1 == 1)
  {
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (!var_elim[i])
      {
        if (!(vc < new_vars))
        {
          printf("BAD ERROR 1\n");
        }
        var_ind[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++)
    {
      if (!con_elim[i])
      {
        if (!(vc < new_cons))
        {
          printf("BAD ERROR 2\n");
        }
        con_ind[vc] = i;
        vc++;
      }
    }
    for (int i = 0; i < new_vars; i++)
    {
      int olda = var_ind[i];
      g_red[i] = g_qpoases[olda];
      for (int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
        H_red[i * new_vars + j] = H_qpoases[olda * num_variables + oldb];
      }
    }

    for (int con = 0; con < new_cons; con++)
    {
      for (int st = 0; st < new_vars; st++)
      {
        float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
        A_red[con * new_vars + st] = cval;
      }
    }

    for (int i = 0; i < new_cons; i++)
    {
      int old = con_ind[i];
      ub_red[i] = ub_qpoases[old];
      lb_red[i] = lb_qpoases[old];
    }

    // Timer solve_timer;
    // solve_timer.start();

    // qpOASES problem
    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);

    // QP initialized

    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    (void)rval;
    // printf("A_red: %.3f", A_red);
    // cout << "A_qpoases:" << &A_qpoases << std::endl;
    // std::cout << "lb_red:" << *lb_red << std::endl;
    // std::cout << "ub_red:" << *ub_red << std::endl;
    // // Stores Solution into q_red
    int rval2 = problem_red.getPrimalSolution(q_red);

    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");

    // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);

    // Reformats solution and stores into q_red
    vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (var_elim[i])
      {
        q_soln[i] = 0.0f;
      }
      else
      {
        q_soln[i] = q_red[vc];
        vc++;
      }
    }

    // std::cout << "Time to solve QP: " << solve_timer.getMs() << " ms" << std::endl;
  }

#ifdef K_PRINT_EVERYTHING
  cout<<"fmat:\n"<<fmat<<endl;
#endif
}