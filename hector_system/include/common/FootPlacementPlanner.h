#include "cppTypes.h"
#include "StateEstimatorContainer.h"
#include "DesiredCommand.h"

// Foot placement plannar based on Linear Inverted Pendulum model
// 
// A = [cosh(wt), sinh(wt)/w, 0, 0;
//      w*sinh(wt), cosh(wt), 0, 0;
//      0, 0, cosh(wt), sinh(wt)/w;
//      0, 0, w*sinh(wt), cosh(wt)]
// B = [1-cosh(wt), 0; 
//      -w*sinh(wt), 0;
//      0, 1-cosh(wt);
//      0, -w*sinh(wt)]
// x_n = A*x_n-1 + B*u_n-1

class LIPController
{
    public:
        LIPController(float reference_height);
        ~LIPController() = default;
        void update_swing_times(double swing_time, double total_swing_time){
            _swing_time = swing_time;
            _total_swing_time = total_swing_time;
        }
        void compute_icp_init(StateEstimate &seResult);
        void compute_icp_final(Vec3<double> support_foot_position);
        Vec2<double> compute_foot_placement(StateEstimate &seResult, DesiredStateData &desiredState, int leg);
        Vec2<double> p; // foot placement
        Vec2<double> x_o;
        Vec2<double> v_o;
        Vec2<double> x_f;
        Vec2<double> v_f;
        Vec2<double> icp_o; 
        Vec2<double> icp_f;
        Vec2<double> b; 
    
    private:
        float _reference_height;
        float _omega;
        double _swing_time;
        double _total_swing_time;
        double _sd;
        double _wd;
};


// Foot placement plannar based on Angular Momentum Linear Inverted Pendulum model
// Based on https://arxiv.org/abs/2109.14862

// 
// A = [0, 0, 0, 1/mzH; 
//      0, 0, -1/mzH, 0; 
//      0, -mg, 0, 0; 
//      mg, 0, 0, 0]
// B = [-1, 0; 
//       0, -1; 
//       0, 0;
//       0, 0]
// x_n = A*x_n-1 + B*u_n-1

// class ALIPController
// {
//     public:
//         ALIPController(float &mass, Eigen::Matrix<float, 3, 3> &I_body, float &reference_height, float sagital_step, float lateral_step);
//         ~ALIPController() = default;
//         void compute_state(StateEstimate &seResult, Vec3<double> & contact_position, DesiredStateData &stateCommand);
//         void solveARE();
//         void LQRControl(Vec3<double> & contact_position);
//         Vec4<double> x; 
//         Vec4<double> x_des;
//         Vec4<double> state;
//         Vec2<double> u; 
//         Vec2<double> u_des;
    
//     private:
//         Eigen::Matrix<double, 4, 4> A;
//         Eigen::Matrix<double, 4, 2> B;
//         Eigen::Matrix<double, 4, 4> Q;
//         Eigen::Matrix<double, 2, 2> R;
//         Eigen::Matrix<double, 4, 4> P; 
//         Eigen::Matrix<double, 4, 4> P_prev;
//         float _mass; 
//         Eigen::Matrix<float, 3, 3> _I_body; // inertial matrix about COM in body frame
//         float _reference_height;
// };