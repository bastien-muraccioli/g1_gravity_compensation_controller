#include "G1GravityCompensationController.h"

G1GravityCompensationController::G1GravityCompensationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {col_zeta_, col_lambda_});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, joint_zeta_, joint_lambda_}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize Task
  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());
  solver().addTask(torqueTask);
  solver().removeTask(getPostureTask());

  datastore().make_call("anchorFrameFunction", 
    [this](const mc_rbdyn::Robot & real_robot) {return createContactAnchor(real_robot);});

  addGui();

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

void G1GravityCompensationController::addGui()
{
  gui()->addElement({"Controller", "CBF"}, 
    mc_rtc::gui::NumberInput("Vel Percent", [this]() {return velPercent_;}, [this](double v) {velPercent_ = v;}),
    mc_rtc::gui::NumberInput("DS Percent", [this]() {return dsPercent_;}, [this](double v) {dsPercent_ = v;}),
    mc_rtc::gui::NumberInput("DI Percent", [this]() {return diPercent_;}, [this](double v) {diPercent_ = v;}),
    mc_rtc::gui::NumberInput("Col Zeta", [this]() {return col_zeta_;}, [this](double v) {col_zeta_ = v;}),
    mc_rtc::gui::NumberInput("Col Lambda", [this]() {return col_lambda_;}, [this](double v) {col_lambda_ = v;}),
    mc_rtc::gui::NumberInput("Joint Zeta", [this]() {return joint_zeta_;}, [this](double v) {joint_zeta_ = v;}),
    mc_rtc::gui::NumberInput("Joint Lambda", [this]() {return joint_lambda_;}, [this](double v) {joint_lambda_ = v;})
  );
}

std::pair<sva::PTransformd, Eigen::Vector3d> G1GravityCompensationController::createContactAnchor(const mc_rbdyn::Robot & anchorRobot)
{
  sva::PTransformd X_foot_r = anchorRobot.bodyPosW("right_ankle_link");
  sva::PTransformd X_foot_l = anchorRobot.bodyPosW("left_ankle_link");

  sva::MotionVecd v_foot_r = anchorRobot.bodyVelW("right_ankle_link");
  sva::MotionVecd v_foot_l = anchorRobot.bodyVelW("left_ankle_link");

  int right_knee_index = int(robot().jointIndexByName("right_knee_joint")) + 5;
  int left_knee_index = int(robot().jointIndexByName("left_knee_joint")) + 5;
  double tau_ext_knee_r =  abs(robot().externalTorques()[right_knee_index]);
  double tau_ext_knee_l =  abs(robot().externalTorques()[left_knee_index]);
  double leftFootRatio = tau_ext_knee_l/(tau_ext_knee_r+tau_ext_knee_l);
  if(tau_ext_knee_r + tau_ext_knee_l < 0.02)
  {
    leftFootRatio = 0.5;
  }
         
  Eigen::VectorXd w_r = X_foot_r.translation();
  Eigen::VectorXd w_l = X_foot_l.translation();
  Eigen::VectorXd contact_anchor = (w_r * (1 - leftFootRatio) + w_l * leftFootRatio)  ;
  Eigen::VectorXd anchor_vel = (v_foot_r.linear() * (1 - leftFootRatio) + v_foot_l.linear() * leftFootRatio);
  contactAnchorTf_ = sva::PTransformd(Eigen::Matrix3d::Identity(), contact_anchor); 

  return {contactAnchorTf_, anchor_vel};
}
