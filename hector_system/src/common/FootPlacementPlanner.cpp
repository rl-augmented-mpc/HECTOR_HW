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
    offset.setZero();
    _swing_time = 0.0;
    _sd = 0.0;
    _wd = 0.0;
    left_foot_pos.setZero();
    right_foot_pos.setZero();
    stance_foot_pos.setZero();
    stance_leg = 0;
}

void LIPController::compute_icp_init(StateEstimate &seResult){
    icp_o << seResult.position(0) + seResult.vWorld(0)/_omega, seResult.position(1) + seResult.vWorld(1)/_omega; //eq2
}

void LIPController::compute_icp_final(){
    // calculate final capture point
    icp_f = std::exp(_omega*_swing_time)*icp_o + (1-std::exp(_omega*_swing_time))*stance_foot_pos; // eq7
}

Vec2<double> LIPController::compute_foot_placement(StateEstimate &seResult, DesiredStateData &desiredState, Vec2<double> foot_placement_residual){
    Vec3<double> v_des_b(desiredState.stateDes[6], desiredState.stateDes[7], 0);
    Vec3<double> v_des_w = seResult.rBody.transpose() * v_des_b;
    double yaw = std::atan2(v_des_w(1), v_des_w(0));

    _sd = (std::sqrt(v_des_w(0)*v_des_w(0) + v_des_w(1)*v_des_w(1))*_total_swing_time +foot_placement_residual[0])*(_swing_time/_total_swing_time); // sagittal step length
    _wd = (step_width+foot_placement_residual[1])*(_swing_time/_total_swing_time); // lateral step width

    b << _sd/(std::exp(_omega*_swing_time)-1), _wd/(std::exp(_omega*_swing_time)+1); // eq9

    offset << -b(0)*std::cos(yaw) - (std::pow(-1,1-stance_leg)*b(1))*std::sin(yaw), -b(0)*std::sin(yaw) + (std::pow(-1,1-stance_leg)*b(1))*std::cos(yaw);
    p = icp_f + offset; // eq.11

    return p;
}