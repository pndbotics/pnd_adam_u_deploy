#ifndef PUBLIC_PARAMENT_H
#define PUBLIC_PARAMENT_H
#include <Eigen/Dense>
#include <vector>

#pragma once
extern int adam_type;
extern int adam_upper_dof_num;
extern int adam_upper_actor_num;

enum ADAM_TYPE { Adam_U = 0, Adam_U_handless = 1 };

typedef enum PndAlgorithmStatusCode {
  PndAlgorithmSuccess = 0,
  PndAlgorithmError = 1,
} PndAlgorithmStatusCode;

struct RobotData {
  // actual position
  Eigen::VectorXd q_a_ = Eigen::VectorXd::Zero(adam_upper_dof_num);
  // actual velocity
  Eigen::VectorXd q_dot_a_ = Eigen::VectorXd::Zero(adam_upper_dof_num);
  // actual torque
  Eigen::VectorXd tau_a_ = Eigen::VectorXd::Zero(adam_upper_dof_num);
  // hand actual position
  Eigen::VectorXd q_a_hands = Eigen::VectorXd::Zero(12);

  // desired position
  Eigen::VectorXd q_d_ = Eigen::VectorXd::Zero(adam_upper_dof_num);
  // desired velocity
  Eigen::VectorXd q_dot_d_ = Eigen::VectorXd::Zero(adam_upper_dof_num);
  // desired torque
  Eigen::VectorXd tau_d_ = Eigen::VectorXd::Zero(adam_upper_dof_num);
  // hand desired position
  Eigen::VectorXd q_d_hands = Eigen::VectorXd::Zero(12);

  // joystick data
  Eigen::VectorXd pCmd_joystick = Eigen::VectorXd::Zero(3);

  // user command

  // log data
  Eigen::VectorXd dataL = Eigen::VectorXd::Zero(200);

  // retarget data
  Eigen::VectorXd retarget_dataL = Eigen::VectorXd::Zero(31);

  bool get_actual_robot_state_flag = true;

  bool retarget_play_flag = false;
  bool retarget_play_flag_continue = false;
  std::string motionselect;
  int motionNumber = 0;
  std::string motionPath;

  bool retarget_record = false;
};

#endif
