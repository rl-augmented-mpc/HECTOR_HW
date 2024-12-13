#ifndef STANDING_H
#define STANDING_H

#include "FSMState.h"
#include "../../PDController/PDController_Stand.h"

class FSMState_PDStand: public FSMState
{
    public:
        FSMState_PDStand(ControlFSMData *data);
        ~FSMState_PDStand(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
    
    private:
        PDController_Stand pdStand;
        Vec3<double> v_des_body = Vec3<double>(0, 0, 0);; // desired body velocity = 0 in x y z
        double turn_rate = 0;
        double pitch = 0;
        double roll = 0;
};

#endif