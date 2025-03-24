/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "../../include/common/Math/Interpolation.h"
#include "../../include/common/FootSwingTrajectory.h"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */

template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase)/swingTime;
  // T phasePI = 2 * M_PI * phase;
  T alpha = 0.8; // it will be analytically solved later
  T phasePI = 2* M_PI * phase * alpha; // early stop curve to deal with slope
  _p = (_pf - _p0) * (phasePI - sin(phasePI))/(2*M_PI) + _p0;
  _v = (_pf - _p0) * (1 - cos(phasePI)) / swingTime;
  T zp, zv;
  zp = _height * (1 - cos(phasePI))/2.0 + _p0[2];
  zv = _height * M_PI * sin(phasePI) / swingTime;

  _p[2] = zp;
  _v[2] = zv;
 
}

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;
