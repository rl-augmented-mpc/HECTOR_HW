#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{//Should I change to GAZEBO_HECTOR, REAL_HECTOR
    GAZEBO_A1,
    REAL_A1
};

enum class UserCommand{
    // EXIT,
    // NONE,
    // START,      // walking
    // L2_B,       // passive
    // L2_X,       // MPC walking
    // L2_A,       // Reserved  
    // L1_A,      // PDStand -> added!
    // WALK,
    // STAND


    NONE,
    START,      // walking
    PASSIVE,       // passive
    WALK,       // MPC walking
    STAND,     // Standing 
    L2_A,       // Reserved  
    PDSTAND,      // PDStand -> added!
    
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    WALKING,
    PASSIVE,
    INVALID,
    PDSTAND,
};


enum class ControlMode{
    PASSIVE,
    PDSTAND,
    STANCE,
    SWING
};


#endif  // ENUMCLASS_H