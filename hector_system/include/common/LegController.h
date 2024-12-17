/*!
 * @file LegController.h
 * @brief Comman Leg Control Interface
 * 
 * Implement low-level leg control for Aliengo Robot
 * Leg 0: FR; Leg 1: FL;
 * Leg 2: RR ; Leg 3: RL;
 */ 

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "Biped.h"

/*!
 * Data sent from control algorithm to legs
 */ 
    struct LegControllerCommand{
        LegControllerCommand() {zero();}

        void zero();

        Vec5<double> qDes, qdDes, tau;
        Vec3<double> pDes, vDes;
        Mat5<double> kpJoint, kdJoint;
        Vec6<double> feedforwardForce;
        Vec3<double> hiptoeforce;
        Mat3<double> kpCartesian;
        Mat3<double> kdCartesian;
        double kptoe;
        double kdtoe;

        int which_control; // To indicate which control mode is being used -> necessary.
        // 0 for none, 1 for PDStand, 2 for stance, 3 for swing
    };

/*!
 * Data returned from legs to control code
 */ 
    struct LegControllerData{
        LegControllerData() {zero();}
        void setBiped(Biped& biped) { hector = &biped; }

        void zero();
        Vec5<double> q, qd;
        Vec3<double> p, v;
        Mat65<double> J;
        Mat35<double> J2;
        Vec5<double> tau;
        Biped* hector;
    };

/*!
 * Controller for 2 legs of hector
 */ 
    class LegController {
      public:
        LegController(Biped& biped) : _biped(biped) {
            for (auto& dat : data) dat.setBiped(_biped);
            for(int i = 0; i<2; i++){
                commands[i].zero();
                data[i].zero();
            }

        };
        
        void zeroCommand();
        void edampCommand(double gain);
        void updateData(const LowlevelState* state);
        void updateCommand(LowlevelCmd* cmd);
        void setEnabled(bool enabled) {_legsEnabled = enabled;};  



        LegControllerCommand commands[2];
        LegControllerData data[2];
        bool _legsEnabled = false;
        std::string limbName[5] = {"Hip 1", "Hip 2", "Thigh", "Knee", "Toe"};
        std::string side[2] = {"Left", "Right"};
        Biped& _biped;


        double JointPDSwitch = 1.0;
        int motiontime = 0;
        
    };

    void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J, Mat35<double>* J2,
                                       Vec3<double>* p, int leg);

    // void computeInverseKinematics(Quadruped& _quad, Vec3<double>& pDes, int leg, Vec3<double>* qDes);

    Vec3<double> InverseKinematics_swingctrl(Vec3<double> &p_Hip2Foot, int leg);

#endif