#include "../../include/common/FootPlacementPlanner.h"

// Foot placement plannar based on Linear Inverted Pendulum model
// https://arxiv.org/abs/2408.02662
// https://github.com/hojae-io/ModelBasedFootstepPlanning-IROS2024/blob/main/gym/envs/humanoid/humanoid_controller.py

LIPController::LIPController(float reference_height)
{
    _reference_height = reference_height;
    _omega = sqrt(9.81/_reference_height);

    p.setZero();
    x_o.setZero();
    v_o.setZero();
    x_f.setZero();
    v_f.setZero();
    icp_o.setZero();
    icp_f.setZero();
    b.setZero();
    _swing_time = 0.0;
    _sd = 0.0;
    _wd = 0.0;
}

void LIPController::compute_icp_init(StateEstimate &seResult){
    _omega = sqrt(9.81/seResult.position(2));
    icp_o(0) = seResult.position(0) + seResult.vWorld(0)/_omega;
    icp_o(1) = seResult.position(1) + seResult.vWorld(1)/_omega;
    x_o(0) = seResult.position(0);
    x_o(1) = seResult.position(1);
    v_o(0) = seResult.vWorld(0);
    v_o(1) = seResult.vWorld(1);
}

void LIPController::compute_icp_final(Vec3<double> support_foot_position){
    // world to support leg frame
    x_o(0) -= support_foot_position(0);
    x_o(1) -= support_foot_position(1);

    // // integrate body position (wrt support leg) based on LIP model
    // x_f(0) = x_o(0)*std::cosh(_omega*_swing_time) + (v_o(0)/_omega)*std::sinh(_omega*_swing_time);
    // v_f(0) = x_o(0)*_omega*std::sinh(_omega*_swing_time) + v_o(0)*std::cosh(_omega*_swing_time);
    // x_f(1) = x_o(1)*std::cosh(_omega*_swing_time) + (v_o(1)/_omega)*std::sinh(_omega*_swing_time);
    // v_f(1) = x_o(1)*_omega*std::sinh(_omega*_swing_time) + v_o(1)*std::cosh(_omega*_swing_time);
    
    // // support leg frame to world frame
    // x_f(0) += support_foot_position(0);
    // x_f(1) += support_foot_position(1);

    // // compute final ICP
    // icp_f(0) = x_f(0) + v_f(0)/_omega;
    // icp_f(1) = x_f(1) + v_f(1)/_omega;

    // Implementation based on paper
    icp_f(0) = std::exp(_omega*_swing_time)*icp_o(0) + (1-std::exp(_omega*_swing_time))*support_foot_position(0);
    icp_f(1) = std::exp(_omega*_swing_time)*icp_o(1) + (1-std::exp(_omega*_swing_time))*support_foot_position(1);
}

Vec2<double> LIPController::compute_foot_placement(StateEstimate &seResult, DesiredStateData &desiredState, int leg){
    double yaw_des = desiredState.stateDes(5);
    Vec3<double> v_des_robot(desiredState.stateDes[6], desiredState.stateDes[7], 0);
    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    _sd = sqrt(std::pow(v_des_world(0),2)+std::pow(v_des_world(1),2)) * _swing_time;
    _wd = 0.22*(_swing_time/_total_swing_time); // lateral step width
    b << _sd/(std::exp(_omega*_swing_time)-1), _wd/(std::exp(_omega*_swing_time)+1);
    p(0) = icp_f(0) - b(0)*std::cos(yaw_des) - std::pow(-1,leg)*b(1)*std::sin(yaw_des);
    p(1) = icp_f(1) - b(0)*std::sin(yaw_des) + std::pow(-1,leg)*b(1)*std::cos(yaw_des);

    return p;
}


// Foot placement plannar based on Angular Momentum Linear Inverted Pendulum model
// Based on https://arxiv.org/abs/2109.14862

// ALIPController::ALIPController(float &mass, Eigen::Matrix<float, 3, 3> &I_body, float &reference_height, float sagital_step, float lateral_step)
// {
//     _mass = mass;
//     _I_body = I_body;
//     float g = 9.81;
//     _reference_height = reference_height;
//     A << 0, 0, 0, 1/(_mass*_reference_height), 
//          0, 0, -1/(_mass*_reference_height), 0, 
//          0, -_mass*g, 0, 0, 
//          _mass*g, 0, 0, 0;
//     B << -1, 0,
//           0, -1,
//           0, 0,
//           0, 0;
//     Q << 1, 0, 0, 0,
//          0, 1, 0, 0,
//          0, 0, 1, 0,
//          0, 0, 0, 1;
//     R << 1, 0,
//          0, 1;
//     P = Q;
//     u_des << sagital_step, lateral_step;
// }

// void ALIPController::compute_state(StateEstimate &seResult, Vec3<double> & contact_position, DesiredStateData &stateCommand)
// {
//     x(0) = seResult.position(0) - contact_position(0);
//     x(1) = seResult.position(1) - contact_position(1);
//     Vec3<double> angular_momentum = _mass* (seResult.position-contact_position).cross(seResult.vWorld) + _I_body*seResult.omegaWorld;
//     x(2) = angular_momentum(0);
//     x(3) = angular_momentum(1);

//     Vec3<double> v_des_world = seResult.rBody.transpose() * stateCommand.stateDes.segment(6, 3);
//     Vec3<double> w_des_world = seResult.rBody.transpose() * stateCommand.stateDes.segment(9, 3);

//     x_des(0) = stateCommand.stateDes(0) - contact_position(0);
//     x_des(1) = stateCommand.stateDes(1) - contact_position(1);
//     Vec3<double> angular_momentum_des = _mass* (stateCommand.stateDes.segment(0, 3) -contact_position).cross(v_des_world) + _I_body*w_des_world;
//     x_des(2) = angular_momentum_des(0);
//     x_des(3) = angular_momentum_des(1);

//     state = x - x_des;
// }

// void ALIPController::solveARE(){
//     int max_iter = 100;
//     double error_thresh = 1e-6;
//     for (int i = 0; i < max_iter; ++i) {
//         P_prev = P;
//         P = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

//         // Check for convergence
//         if ((P - P_prev).norm() < error_thresh) {
//             break;
//         }
//     }
// }

// void ALIPController::LQRControl(Vec3<double> & contact_position){
//     // u = p_c + u_ref - K*x
//     u = contact_position.segment(0,2) - R.inverse() * B.transpose() * P * state;
// }