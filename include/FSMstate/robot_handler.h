#ifndef ROBOT_HANDLER_H_
#define ROBOT_HANDLER_H_

#include <memory>
#include "fsm_state.h"
#include "joystick.h"

class Framework {
 public:
  explicit Framework(RobotData &robot_data);
  ~Framework();
  bool init();
  void entryStop();

  void runFSM();
  FSMStateName getCurrentState();
  bool disableJoints;

 private:
  FSMState *getNextState(FSMStateName stateName) const;

 private:
  std::unique_ptr<RobotCommon> robot_common_;
  std::vector<int> leg_joint_ids_;
  Eigen::VectorXd leg_pos_;
  Eigen::VectorXd leg_vel_;
  Eigen::VectorXd leg_tau_;

  StateList state_list_;
  FSMState *current_state_;
  FSMState *next_state_;
  FSMStateName next_state_name_;

  bool first_run_;
};

#endif  // ROBOT_HANDLER_H_
