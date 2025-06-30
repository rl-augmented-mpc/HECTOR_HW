#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "../hector_system/include/common/ControlFSMData.h"
#include "../hector_system/include/FSM/FSM.h"
#include "../hector_system/include/common/ContactEstimator.h"
#include "../hector_system/include/common/OrientationEstimator.h"

#include "../Interface/HW_interface/include/interface/IOSDK.h"
#include "../Interface/HW_interface/include/interface/WirelessHandle.h"
#include "../Interface/HW_interface/include/stateestimator/PositionVelocityEstimator.h"


using namespace UNITREE_LEGGED_SDK;

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if(sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
      std::cout << "[ERROR] function setprocessscheduler failed \n ";  
    }
}

int main()
{
    setProcessScheduler();
    
    double dt = 0.001;    
    int robot_id = 4; // AlienGo=1, A1=2, Biped=4
    int cmd_panel_id = 2; // Wireless=1, keyboard=2 (ONLY keyboard is supported for now)

    IOSDK* ioInter = new IOSDK(LeggedType::A1, 2);
    Biped biped;
    biped.setBiped(1);


    LegController* legController = new LegController(biped);
    LowlevelCmd* lowCmd = new LowlevelCmd();
    LowlevelState* lowState = new LowlevelState();

    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(lowState,
                                                                          legController->data,
                                                                          &stateEstimate);
                                                                        
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    // stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();  
    stateEstimator->addEstimator<T265TrackingCameraEstimator>();


    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_lowCmd = lowCmd;
    _controlData->_lowState = lowState;

    // gait parameters
    // Vec2<int> dsp_durations = {0, 0};
    // Vec2<int> ssp_durations = {5, 5};
    Vec2<int> dsp_durations = {3, 3}; // 0.05*3 = 0.15s
    Vec2<int> ssp_durations = {3, 3}; // 0.05*3 = 0.15s
    biped.updateGaitParameter(dsp_durations, ssp_durations);
    double slope_angle = (15.0 / 180.0) * M_PI;
    biped.updateSlope(slope_angle);
    std::string planner = "Raibert"; // or LIP
    biped.setFootPlacementPlanner(planner);

    // MPC sampling time
    // double dt_mpc = 0.045; 
    double dt_mpc = 0.05; 
    biped.rl_params.update_sampling_dt(dt_mpc);
    
    std::string fsm_name = "passive";
    int iterations_between_mpc = 50; // this does not have any effect
    int horizon_length = 10;
    int mpc_decimation = 5; // 1000/5 = 200Hz
    FSM* _FSMController = new FSM(_controlData, dt, iterations_between_mpc, horizon_length, mpc_decimation, fsm_name);
    ioInter->_data = _controlData;


    LoopFunc loop_control("control_loop", dt, boost::bind(&FSM::run, _FSMController));
    LoopFunc loop_udpSend("udp_send",     dt, 3, boost::bind(&IOSDK::sendRecv, ioInter));


    loop_udpSend.start();
    loop_control.start();


    while(1){
        +sleep(10);
    }
    
    delete _FSMController;
    delete _controlData;

    return 0;

}
