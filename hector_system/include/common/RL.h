// Reinforcement Learning based Residual Learning parameters

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include "cppTypes.h"

class RL {

    public:
        RL(){
            A_residual.setZero();
            B_residual.setZero();
            delta_grw.setZero();
            delta_foot_placement.setZero();
            delta_joint_position.setZero();
        }
        ~RL() = default;

        void set_residual_dynamics(Eigen::Matrix<float, 13, 13> _A_residual, Eigen::Matrix<float, 13, 12> _B_residual){
            A_residual = _A_residual;
            B_residual = _B_residual;
        }

        void set_residual_foot_placement(Vec4<double> _delta_foot_placement){
            delta_foot_placement = _delta_foot_placement;
        }

        void set_residual_joint_position(Vec10<double> _delta_joint_position){
            delta_joint_position = _delta_joint_position;
        }

        void update_sampling_dt(double dt_sampling){
            _dt_sampling = dt_sampling;
        }

        Eigen::Matrix<float, 13, 13> A_residual; 
        Eigen::Matrix<float, 13, 12> B_residual;
        Vec12<double> delta_grw; 
        Vec4<double> delta_foot_placement; 
        Vec10<double> delta_joint_position; 
        double _dt_sampling= 0.02;

};