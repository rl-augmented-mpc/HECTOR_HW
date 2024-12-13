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
    NONE,
    START,      // walking
    L2_B,       // passive
    L2_X,       // MPC walking
    L2_A,       // Reserved  
    WALK,      // walking (gait)
    STAND      // standing (gait)
    
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
};


#endif  // ENUMCLASS_H