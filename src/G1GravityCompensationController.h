#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>

#include <algorithm>
#include <map>

#include "api.h"

struct G1GravityCompensationController_DLLAPI G1GravityCompensationController : public mc_control::fsm::Controller
{
  G1GravityCompensationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::TorqueTask> torqueTask;

private:
  void addGui();
  void updateDampingTarget();
  mc_rtc::Configuration config_;
  // Joint damping added through the torque task target: tau_d = -damping_ * qdot [Nm.s/rad]
  double damping_ = 0.5;
  std::map<std::string, std::vector<double>> dampingTarget_;
  std::pair<sva::PTransformd, Eigen::Vector3d> createContactAnchor(const mc_rbdyn::Robot & anchorRobot);
  sva::PTransformd contactAnchorTf_;
  // Constraint configuration
  double velPercent_ = 0.95;
  double dsPercent_ = 0.01;
  double diPercent_ = 0.1;
  // CBF parameters
  double col_zeta_ = 1.2;
  double col_lambda_ = 60.0;
  double joint_zeta_ = 1.2;
  double joint_lambda_ = 120.0;
};
