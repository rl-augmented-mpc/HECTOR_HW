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
        void update_stance_leg(int leg, Vec2<double> stance_foot_position){
            stance_leg = leg; 
            stance_foot_pos = stance_foot_position;
        };
        void update_swing_times(double swing_time, double total_swing_time){
            _swing_time = swing_time;
            _total_swing_time = total_swing_time;
        };
        void compute_icp_init(StateEstimate &seResult);
        void compute_icp_final();
        Vec2<double> compute_foot_placement(StateEstimate &seResult, DesiredStateData &desiredState, Vec2<double> foot_placement_residual);

        int stance_leg; // 0 for left, 1 for right
        Vec2<double> p; // foot placement

        // com init state wrt stance foot
        Vec2<double> x_o;
        Vec2<double> v_o;
        // com final state wrt stance foot
        Vec2<double> x_f;
        Vec2<double> v_f;

        // com state wrt world frame
        Vec2<double> x_f_w; 

        // instantaneous capture point
        Vec2<double> icp_o; 
        Vec2<double> icp_f;

        // step length
        Vec2<double> b; 
        Vec2<double> offset; 

        // bookkeeping variables
        Vec2<double> left_foot_pos; 
        Vec2<double> right_foot_pos;
        Vec2<double> stance_foot_pos; 
    
    private:
        float _reference_height;
        float _omega;
        double _swing_time; // swing time remaining
        double _total_swing_time; // swing duration
        double _sd;
        double _wd;
        double step_width = 0.05; // baseline step width
};