#include "G1GravityCompensationController.h"

G1GravityCompensationController::G1GravityCompensationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 60.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, 1.2, 120.0}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize Task
  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());
  solver().addTask(torqueTask);
  solver().removeTask(FSMPostureTask);

  datastore().make_call("anchorFrameFunction", 
    [this](const mc_rbdyn::Robot & real_robot) {return createContactAnchor(real_robot);});

  mc_rtc::log::success("G1GravityCompensationController init done ");
}

bool G1GravityCompensationController::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void G1GravityCompensationController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
