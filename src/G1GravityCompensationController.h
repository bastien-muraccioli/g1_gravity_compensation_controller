#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>

#include "api.h"

struct G1GravityCompensationController_DLLAPI G1GravityCompensationController : public mc_control::fsm::Controller
{
  G1GravityCompensationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::TorqueTask> torqueTask;

private:
  mc_rtc::Configuration config_;
};
