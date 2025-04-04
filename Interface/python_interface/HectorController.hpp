#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <thread>
#include <memory>

#include "../../hector_system/include/common/ControlFSMData.h"
#include "../../hector_system/include/FSM/FSM.h"


class HectorController{
    public:
        HectorController(
            double _dt, int _iterations_between_mpc, int _horizon_length, int _mpc_decimation)
        {
            std::cout << "==== CPP INFO ====" << std::endl;
            biped.setBiped(0);
            legController = std::make_shared<LegController>(biped);
            cmd = std::make_shared<LowlevelCmd>();
            state = std::make_shared<LowlevelState>();

            std::cout << "setup state estimator " << std::endl;
            stateEstimator = std::make_shared<StateEstimatorContainer>(state.get(), legController->data, &stateEstimate);
            desiredStateCommand = std::make_shared<DesiredStateCommand>(&stateEstimate, _dt);

            std::cout << "setup control fsm data " << std::endl;
            _controlData = std::make_shared<ControlFSMData>();
            _controlData->_biped = &biped;
            _controlData->_stateEstimator = stateEstimator.get();
            _controlData->_legController = legController.get();
            _controlData->_desiredStateCommand = desiredStateCommand.get();
            _controlData->_lowCmd = cmd.get();
            _controlData->_lowState = state.get();

            std::cout << "setup fsm " << std::endl;
            std::string fsm_name = "walking";
            fsm = std::make_shared<FSM>(_controlData.get(), _dt, _iterations_between_mpc, _horizon_length, _mpc_decimation, fsm_name);
            std::cout << "=============" << std::endl;
        }

        ~HectorController() = default;

        void reset(){
            fsm->reset();
        }

        void setFrictionCoefficient(float mu){
            biped.setFrictionCoefficient(mu);
        }

        void setFootPlacementPlanner(std::string planner_name){
            biped.setFootPlacementPlanner(planner_name);
        }

        void setGaitNum(int gaitnum){
            // 1: standing
            // 2: walking
            fsm->setGaitNum(gaitnum);
        }

        void updateGaitParameter(Vec2<int> dsp_durations, Vec2<int> ssp_durations){
            biped.updateGaitParameter(dsp_durations, ssp_durations);
        }

        // Set target roll_pitch, 2D twist (vx, vy, wz), and height
        void setTargetCommand(Vec2<double> roll_pitch, Vec3<double> twist, double ref_height)
        {
            fsm->setStateCommands(roll_pitch, twist, ref_height);
        }

        // set ground truth state to state estimator
        void setState(Vec3<double>position, Quat<double>orientation, Vec3<double>vBody, Vec3<double>omegaBody, Vec10<float> joint_position, Vec10<float> joint_velocity)
        {

            // set state estimation data
            stateEstimator->setStateEstimate(position, orientation, vBody, omegaBody);

            // set motor state
            for (int i=0;i<10;i++){
                state->motorState[i].q = joint_position[i];
                state->motorState[i].dq = joint_velocity[i];
            }
        }

        void setGaitSteppingFrequency(float stepping_frequency){
            biped.setSteppingFrequency(stepping_frequency);
        }

        void setFootHeight(float foot_height){
            biped.setFootHeight(foot_height);
        }

        void setSlopePitch(float slope_pitch){
            biped.updateSlope(slope_pitch);
        }

        void setSRBDResidual(Eigen::Matrix<float, 13, 13> A_residual, Eigen::Matrix<float, 13, 12> B_residual){
            biped.rl_params.set_residual_dynamics(A_residual, B_residual);
        }

        void run(){
            fsm->run();
        }

        void computeAction(){
            for (int i=0;i<10;i++){
                ff_torque(i) = cmd->motorCmd[i].tau;
                refpos(i) = cmd->motorCmd[i].q;
                refvel(i) = cmd->motorCmd[i].dq;
                torque(i) = cmd->motorCmd[i].tau + 
                            cmd->motorCmd[i].Kp * (cmd->motorCmd[i].q-state->motorState[i].q) +
                            cmd->motorCmd[i].Kd * (cmd->motorCmd[i].dq-state->motorState[i].dq);
                torque(i) = std::min(std::max(torque(i), -1* biped.torque_limit[i]), biped.torque_limit[i]);
            }
        }

        void updateGRFM(Vec12<double> grfm){
            legController->update_grw(grfm);
        }

        void addResidualGRFM(Vec12<double> delta_grfm){
            legController->add_residual_grw(delta_grfm);
        }

        void updateLowLevelCommand(){
            legController->updateCommandResidual(cmd.get());
        }

        void addResidualFootPlacement(Vec4<double> delta_foot_placement){
            biped.rl_params.set_residual_foot_placement(delta_foot_placement);
        }

        void addResidualJointPosition(Vec10<double> delta_joint_position){
            legController->add_residual_joint_position(delta_joint_position);
        }

        /// **** helper functions to get access to the computed values **** ///

        Vec10<float> getFFTorque(){
            return ff_torque;
        }

        Vec10<float> getRefPos(){
            return refpos;
        }

        Vec10<float> getRefVel(){
            return refvel;
        }

        Vec10<float> getTorque(){
            return torque;
        }

        Vec12<double> getGRFM(){
            return legController->get_grw();
        }

        Vec4<double> getReibertFootPlacement(){
            return legController->get_reibert_foot_placement();
        }

        Vec4<double> getFootPlacement(){
            return legController->get_foot_placement();
        }

        Vec6<double> getRefFootPosition(){
            return legController->get_ref_swing_position();
        }

        Vec6<double> getFootPosition(){
            return legController->get_swing_position();
        }

        Vec2<double> getContactPhase(){
            return legController->get_contact_phase();
        }

        Vec2<double> getSwingPhase(){
            return legController->get_swing_phase();
        }

        Vec2<double> getContactState(){
            return legController->get_contact_state();
        }

        Vec2<double> getSwingState(){
            return legController->get_swing_state();
        }
    
    private:
        double system_dt; 
        int iterations_between_mpc;
        int horizonLength;

        Biped biped;
        StateEstimate stateEstimate;

        std::shared_ptr<LowlevelCmd> cmd;
        std::shared_ptr<LowlevelState> state;
        std::shared_ptr<LegController> legController;
        std::shared_ptr<StateEstimatorContainer> stateEstimator;
        std::shared_ptr<DesiredStateCommand> desiredStateCommand;
        std::shared_ptr<ControlFSMData> _controlData;
        std::shared_ptr<FSM> fsm;

        Vec10<float> torque;
        Vec10<float> ff_torque;
        Vec10<float> refpos;
        Vec10<float> refvel;    

}; 