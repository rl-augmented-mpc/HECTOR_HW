/*!
 * @file Quadruped.h
 * @brief stores dynamics information
 * Leg 0: Left front; Leg 1: right front;
 * Leg 2: Left rear ; Leg 3: right rear;
 */ 
#ifndef PROJECT_QUADRUPED_H
#define PROJECT_QUADRUPED_H

#include <vector>
#include "cppTypes.h"
class Quadruped{
  public:
    void setQuadruped(int robot_id){
        
        mass = 13.9;

        leg_offset_x = 0.0;
        leg_offset_y = -0.047;//0.057;
        leg_offset_z = -0.1265;//-0.125;

        leg_offset_x2 = 0.0;
        leg_offset_y2 = -0.047;//0.057;
        leg_offset_z2 = -0.1365;

        hipLinkLength = 0.06; // hip offset in const.xacro
        thighLinkLength = 0.22;
        calfLinkLength = 0.22;
    }
    int robot_index; // 1 for Aliengo, 2 for A1
    double hipLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double leg_offset_x2;
    double leg_offset_y2;
    double leg_offset_z2;
    double mass;
    Vec3<double> getHipLocation(int leg){
        assert(leg >=0 && leg <4);
        Vec3<double> pHip = Vec3<double>::Zero();
        leg_offset_x = 0.0;
        leg_offset_y = -0.047;//0.057;
        leg_offset_z = -0.1265;//-0.125;

        leg_offset_x2 = 0.0;
        leg_offset_y2 = -0.047;//0.057;
        leg_offset_z2 = -0.1365;

        hipLinkLength = 0.06; // hip offset in const.xacro
        thighLinkLength = 0.22;
        calfLinkLength = 0.22;
        if (leg == 0){
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1){
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }

        return pHip;
    };

    Vec3<double> getHip2Location(int leg){  //Todo, not getting offset data, fix later
        assert(leg >=0 && leg <2);
        Vec3<double> pHip2 = Vec3<double>::Zero();
        double leg_offset_x = 0.0;
        double leg_offset_y = -0.047;//0.057;
        double leg_offset_z = -0.1265;//-0.125;
        double leg_offset_x2 = 0.0;
        double leg_offset_y2 = -0.047;//0.057;
        double leg_offset_z2 = -0.1365;

        double hipLinkLength = 0.06; // hip offset in const.xacro
        double thighLinkLength = 0.22;
        double calfLinkLength = 0.22;
        if (leg == 0){
            pHip2(0) = leg_offset_x2;
            pHip2(1) = leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        if (leg == 1){
            pHip2(0) = leg_offset_x2;
            pHip2(1) = -leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        return pHip2;
    };

};

#endif
