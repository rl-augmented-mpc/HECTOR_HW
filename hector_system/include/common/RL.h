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

        void setFootHeight(double _foot_height){
            foot_height = _foot_height;
        }
    
        void setSteppingFrequency(double _gait_stepping_frequency){
            gait_stepping_frequency = _gait_stepping_frequency;
        }
    
        void setSwingFootControlPoint(double _cp1_coef, double _cp2_coef){
            cp1_coef = _cp1_coef;
            cp2_coef = _cp2_coef;
        }

        void setSwingDuration(double _swing_duration){
            swing_duration = _swing_duration;
        }

        void updateGaitParameter(Vec2<int> _dsp_durations, Vec2<int> _ssp_durations){
            dsp_durations = _dsp_durations;
            ssp_durations = _ssp_durations;
        }

        Eigen::Matrix<float, 13, 13> A_residual; 
        Eigen::Matrix<float, 13, 12> B_residual;
        Vec12<double> delta_grw; 
        Vec4<double> delta_foot_placement; 
        Vec10<double> delta_joint_position; 

        // parameters for reference and swing leg controller
        double foot_height=0.12; // swing foot height
        float gait_stepping_frequency = 1.0; // gait stepping frequency
        double cp1_coef = 0.33;
        double cp2_coef = 0.66;
        double swing_duration = 0.2; 

        Vec2<int> dsp_durations = {0, 0}; 
        Vec2<int> ssp_durations = {int(0.2/0.001), int(0.2/0.001)};

};