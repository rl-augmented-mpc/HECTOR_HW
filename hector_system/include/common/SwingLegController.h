#ifndef SWINGLEGCONTROLLER_H
#define SWINGLEGCONTROLLER_H
#include "../../ConvexMPC/GaitGenerator.h"
#include "../../include/common/ControlFSMData.h"
#include "../../include/common/FootSwingTrajectory.h"
#include "../../include/common/cppTypes.h"
#include "../../include/common/LegController.h"
#include "../../include/common/FootPlacementPlanner.h"


 /**
  * @note varibles with _w are in world frame
  * @note varibles with _b are in body frame
 */
class swingLegController {
    public:
        static constexpr int nLegs = 2;
    
        swingLegController();
        ~swingLegController() = default;

        /**
         * @brief Initialize the swing leg controller
         * @param data: pointer to the control data
         * @param gait: pointer to the gait generator
         * @param dtSwing: time step for the swing leg controller
         * @note This function is an alternative to the constructor in case 
         *       the gait generator and control data are not available at 
         *       the time of construction
        */
        void initSwingLegController(ControlFSMData *_data, Gait *_gait, double dtSwing);
        void setPlanner(string planner_name){
            plannar = planner_name;
        }
        void setGait(Gait *_gait);
        void reset(){
            firstSwing[0] = true;
            firstSwing[1] = true;
            Pf_0[0].setZero();
            Pf_0[1].setZero();
            Pf[0].setZero();
            Pf[1].setZero();
            Pf_augmented[0].setZero();
            Pf_augmented[1].setZero();
        }
        
        /**
         * @brief Update the swing leg controller
         * @note This function should be called at every control loop iteration
         */
        void updateSwingLeg();
        void updateFootPlacementPlanner();
        void updateSwingFootCommand();
        void setSteppingFrequency(double stepping_frequency) { _stepping_frequency = stepping_frequency; }
        void setFootHeight(double foot_height) { _foot_height = foot_height; }
        
        /**
         * @brief Compute an approximate inverse kinematics for 5-DoF swing leg
         * @param bodyPositionDesired: desired position of the end effector in the body frame
         * @param leg: leg index (0 for left, 1 for right)  
         * @param jointAngles: output joint angles
        */
        void setFootplacementResidual(Vec2<double> pf_residual, int foot);
        Vec3<double> getReibertFootPlacement(int foot);
        Vec3<double> getAugmentedFootPlacement(int foot);
        std::array<Vec3<double>, 10> getReferenceSwingFootPosition();
        

    private:
        LIPController lip_controller;
        Vec2<double> lip_foot_placement;
        Gait* gait;
        const ControlFSMData* data;
        StateEstimate seResult;
        double _dt; 
        double _dtSwing;
        FootSwingTrajectory<double> footSwingTrajectory[nLegs];
        Vec3<double> pFoot_w[nLegs];
        Vec3<double> vFoot_w[nLegs]; 
        Vec3<double> pFoot_b[nLegs];
        Vec3<double> vFoot_b[nLegs];                        
        Vec2<double> swingStates;
        Vec2<double> contactStates;
        Vec2<double> swingTimes;
        Vec5<double> qDes[nLegs];
        Vec3<double> Pf_0[nLegs];
        Vec3<double> Pf[nLegs];
        Vec3<double> Pf_augmented[nLegs];
        Vec2<double> Pf_residual[nLegs];
        bool firstSwing[nLegs] = {true, true};
        string plannar;
        
        void updateFootPosition();
        void updateSwingStates();
        void updateSwingTimes();
        void computeFootPlacement();
        void computeFootDesiredPosition();
        void setDesiredJointState();

        // swing leg related
        double _foot_height = 0.2; 
        double _stepping_frequency = 1.0;

        // utility functions
        double clamp(double val, double minVal, double maxVal) {
                return std::max(minVal, std::min(val, maxVal));
        }                            

}; // class swingLegController

#endif // SWINGLEGCONTROLLER_H    