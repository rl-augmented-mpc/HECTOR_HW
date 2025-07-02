#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "HectorController.hpp"

namespace py = pybind11;

PYBIND11_MODULE(hector_control, m) {
  py::class_<HectorController>(m, "HectorController")
      .def(py::init<double, int, int, int>())

      // initial setups
      .def("switch_fsm", &HectorController::switch_fsm, py::arg("fsm_name"))
      .def("reset", &HectorController::reset)
      .def("setGaitNum", &HectorController::setGaitNum, py::arg("gaitnum"))
      .def("updateGaitParameter", &HectorController::updateGaitParameter, py::arg("dsp_durations"), py::arg("ssp_durations"))
      .def("setFrictionCoefficient", &HectorController::setFrictionCoefficient, py::arg("mu"))
      .def("setFootPlacementPlanner", &HectorController::setFootPlacementPlanner, py::arg("planner_name"))
      .def("setTargetCommand", &HectorController::setTargetCommand, py::arg("roll_pitch"), py::arg("twist"), py::arg("ref_height"))

      // running controller
      .def("setState", &HectorController::setState, py::arg("position"), py::arg("orientation"), py::arg("vBody"), py::arg("omegaBody"), py::arg("joint_position"), py::arg("joint_velocity"))
      .def("run", &HectorController::run)
      .def("computeAction", &HectorController::computeAction)

      // update parameters
      .def("updateSamplingTime", &HectorController::updateSamplingTime, py::arg("dt_sampling"))
      .def("setGaitSteppingFrequency", &HectorController::setGaitSteppingFrequency, py::arg("stepping_frequency"))
      .def("setFootHeight", &HectorController::setFootHeight, py::arg("foot_height"))
      .def("setSwingFootControlPoint", &HectorController::setSwingFootControlPoint, py::arg("cp1_coef"), py::arg("cp2_coef"))
      .def("setFootPlacementZ", &HectorController::setFootPlacementZ, py::arg("pf_z"))
      .def("setSRBDResidual", &HectorController::setSRBDResidual, py::arg("A_residual"), py::arg("B_residual"))
      .def("setSlopePitch", &HectorController::setSlopePitch, py::arg("slope_pitch"))
      .def("updateGRFM", &HectorController::updateGRFM, py::arg("grfm"))
      .def("addResidualGRFM", &HectorController::addResidualGRFM, py::arg("delta_grfm"))
      .def("addResidualFootPlacement", &HectorController::addResidualFootPlacement, py::arg("delta_foot_placement"))
      .def("addResidualJointPosition", &HectorController::addResidualJointPosition, py::arg("delta_joint_position"))
      .def("updateLowLevelCommand", &HectorController::updateLowLevelCommand)

      // getting results
      .def("getRefPos", &HectorController::getRefPos)
      .def("getRefVel", &HectorController::getRefVel)
      .def("getFFTorque", &HectorController::getFFTorque)
      .def("getTorque", &HectorController::getTorque)
      .def("getGRFM", &HectorController::getGRFM)
      .def("getContactPhase", &HectorController::getContactPhase)
      .def("getSwingPhase", &HectorController::getSwingPhase)
      .def("getContactState", &HectorController::getContactState)
      .def("getSwingState", &HectorController::getSwingState)
      .def("getReibertFootPlacement", &HectorController::getReibertFootPlacement)
      .def("getFootPlacement", &HectorController::getFootPlacement)
      .def("getRefFootPosition", &HectorController::getRefFootPosition)
      .def("getFootPosition", &HectorController::getFootPosition)
      .def("getCost", &HectorController::getCost)
      .def("getReferencePositionTrajectory", &HectorController::getReferencePositionTrajectory)
      .def("getReferenceOrientationTrajectory", &HectorController::getReferenceOrientationTrajectory)
      .def("getReferenceFootPositionTrajectory", &HectorController::getReferenceFootPositionTrajectory);

      
}