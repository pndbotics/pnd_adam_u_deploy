#include "robot_handler.h"

#include <sys/resource.h>

#include <fstream>
#include <iostream>

Framework::Framework(RobotData &robot_data) {
  FSMInit();
  
  state_list_.zero = new StateZero(&robot_data);
  state_list_.teleoperation =  new StateTeleoperation(&robot_data);
  state_list_.stop = new StateStop(&robot_data);

  first_run_ = true;
  current_state_ = state_list_.stop;
  next_state_ = state_list_.stop;
  next_state_name_ = FSMStateName::STOP;
}

Framework::~Framework() {
  delete state_list_.zero;
  delete state_list_.teleoperation;
  delete state_list_.stop;
}

bool Framework::init() {
  return true;
}

void Framework::entryStop() {}

void Framework::runFSM() {
  if (first_run_) {
    current_state_->onEnter();
    first_run_ = false;
    std::cout << "FSM start!" << std::endl;
    std::cout << "current state: " << current_state_->current_state_name_ << std::endl;
  }

  next_state_name_ = current_state_->checkTransition();
  if (next_state_name_ != current_state_->current_state_name_) {
    next_state_ = getNextState(next_state_name_);

    current_state_->onExit();

    current_state_ = next_state_;
    current_state_->onEnter();

  }
  current_state_->run();
}

FSMState *Framework::getNextState(FSMStateName stateName) const {
  switch (stateName) {
    case FSMStateName::ZERO:
      return state_list_.zero;
    case FSMStateName::TELEOPERATION:
      return state_list_.teleoperation;
    case FSMStateName::STOP:
      return state_list_.stop;
    default:
      return state_list_.stop;
  }
}
