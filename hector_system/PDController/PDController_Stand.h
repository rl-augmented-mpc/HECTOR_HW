#ifndef PDCONTROLLER_STAND_H
#define PDCONTROLLER_STAND_H

#include <Eigen/Dense>
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"
#include <fstream>

class PDController_Stand {
public:
    // Constructors
    PDController_Stand();

    Vec5<double> q_target;
    Vec5<double> q_curr;
    Vec5<double> error;
    Vec2<Vec5<double>> prevError;
    Vec5<double> controlSignal;

    Vec5<double> kp;
    Vec5<double> kd;

    double dt;
    bool firstRun;

    void run(ControlFSMData& data);
};

#endif
