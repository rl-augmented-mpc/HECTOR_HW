/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "../../include/common/Math/Interpolation.h"
#include "../../include/common/FootSwingTrajectory.h"
#include <iostream>

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */

template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {

  // Cycloid trajectory
  // _pf[2] = _p0[2];
  // T phasePI = 2 * M_PI * phase;
  // T zp, zv;
  // _p = (_pf - _p0) * (phase - sin(phasePI)/(2*M_PI)) + _p0;
  // _v = (_pf - _p0) * (1 - cos(phasePI)) / swingTime;
  // zp = (_height/2) * (1 - cos(phasePI)) + _p0[2];
  // zv = _height * M_PI * sin(phasePI) / swingTime;
  // _p[2] = zp;
  // _v[2] = zv;

  // // parametric cubic bezier
  Vec3<T> _p1 = _p0 + (_pf - _p0)*_cp1_coef;
  Vec3<T> _p2 = _p0 + (_pf - _p0)*_cp2_coef;

  // assuming foot is at apex at t=0.5, you can find p1 and p2 from apex height
  T z_apex = _p0[2] + _height;
  _p1[2] = (T(8)* z_apex - _p0[2] - _pf[2]) / T(6);
  _p2[2] = (T(8)* z_apex - _p0[2] - _pf[2]) / T(6);

  _p = std::pow(1-phase, 3) * _p0 + 
      3 * std::pow(1-phase, 2) * phase * _p1 +
      3 * (1-phase) * std::pow(phase, 2) * _p2 +
      std::pow(phase, 3) * _pf;

  _v = (3 * std::pow(1-phase, 2) * (_p1 - _p0) +
        6 * (1-phase) * phase * (_p2 - _p1) +
        3 * std::pow(phase, 2) * (_pf - _p2)) / swingTime;
 
}

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;
