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

#include <iomanip>
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
        Vec5<double> kpJoint, kdJoint;
        Vec6<double> feedforwardForce;
        Vec3<double> hiptoeforce;
        Mat3<double> kpCartesian;
        Mat3<double> kdCartesian;
        double kptoe;
        double kdtoe;
    
        // for residual learning
        Vec5<double> qDesDelta; 
        Vec6<double> feedforwardForceDelta;
        Vec2<double> footplacementDelta; 
        Vec3<double> Pf_world; 
        Vec3<double> Pf_base; 
        Vec3<double> Pf_augmented;
        double contact_phase; 
        double contact_state; 
        double swing_phase; 
        double swing_state;

        int control_mode = 0; // To indicate which control mode is being used -> necessary.
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


        int motiontime = 0;

        // helper methods for residual learning
        // get access to GRF-M and foot positions
        Vec12<double> get_grw(); 
        Vec2<double> get_contact_phase();
        Vec2<double> get_swing_phase();
        Vec2<double> get_contact_state();
        Vec2<double> get_swing_state();
        Vec6<double> get_foot_placement(); 
        Vec6<double> get_foot_placement_base(); 
        Vec6<double> get_ref_swing_position();
        Vec6<double> get_swing_position();

        // update parameters
        void update_grw(Vec12<double> grfm);
        void add_residual_grw(Vec12<double> delta_grfm);
        void add_residual_foot_placement(Vec4<double> delta_foot_placement);
        void add_residual_joint_position(Vec10<double> delta_joint_position);
        void updateCommandResidual(LowlevelCmd* cmd); 

        // bookkeeping
        Vec12<double> grfm = Vec12<double>::Zero();
        Vec10<double> qref = Vec10<double>::Zero();
        Vec6<double> Pfs_base = Vec6<double>::Zero();
        Vec6<double> Pfs = Vec6<double>::Zero();
        Vec6<double> foot_ref_pos = Vec6<double>::Zero();
        Vec6<double> foot_pos = Vec6<double>::Zero();
        Vec2<double> contact_phase= Vec2<double>::Zero();
        Vec2<double> swing_phase= Vec2<double>::Zero();
        Vec2<double> contact_state = Vec2<double>::Zero();
        Vec2<double> swing_state = Vec2<double>::Zero();

        
    };

    void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J, Mat35<double>* J2,
                                       Vec3<double>* p, int leg);

    Vec3<double> InverseKinematics_swingctrl(Vec3<double> &p_Hip2Foot, int leg);

#endif