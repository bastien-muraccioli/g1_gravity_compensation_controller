#include "G1GravityCompensationController_Initial.h"

#include "../G1GravityCompensationController.h"

void G1GravityCompensationController_Initial::configure(const mc_rtc::Configuration & config) {}

void G1GravityCompensationController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1GravityCompensationController &>(ctl_);
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
}

bool G1GravityCompensationController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1GravityCompensationController &>(ctl_);
  
  return false;
}

void G1GravityCompensationController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1GravityCompensationController &>(ctl_);
}

EXPORT_SINGLE_STATE("G1GravityCompensationController_Initial", G1GravityCompensationController_Initial)
