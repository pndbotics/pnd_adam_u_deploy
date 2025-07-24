
#ifndef FSM_STATE_H_
#define FSM_STATE_H_

#include <time.h>
#include "robot_common.h"
#include "joystick.h"
#include <deque>
#include "public_parament.h"
#include "basic_function.h"

#include <dirent.h>
#include <fstream>
#include <sstream>

void FSMInit();

class StateZero : public FSMState
{
public:
  explicit StateZero(RobotData *robot_data);

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

private:
  const double total_time = 2.0;

  bool zero_finish_flag = false;
  Eigen::VectorXd init_joint_pos = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd zero_ = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd joint_pos_ = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd joint_vel_ = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd joint_acc_ = Eigen::VectorXd::Zero(adam_upper_actor_num);
};

class StateTeleoperation : public FSMState
{
public:
  explicit StateTeleoperation(RobotData *robot_data);
  ~StateTeleoperation() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

private:
  void retarget_teleoperation();
  void retarget_motion_play();
  void retarget_motion_record();

  Eigen::VectorXd upperJointsRefPosFilter = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd upperJointsRefPosLast = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd upperJointsCmd = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd upperJointsRefPos = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd upperJointsRefVel = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::VectorXd upperJointsRefPosFilterLast = Eigen::VectorXd::Zero(adam_upper_actor_num);
  Eigen::MatrixXd upper_joints_x_d = Eigen::MatrixXd::Zero(4, adam_upper_actor_num);

  double time1, time2, time3, time4;
  double totaltime = 2.0;
  bool motion_start_flag = true;
  bool mocap_file_ok = true;

  int mocap_motion_line_num;
  std::ifstream infile;
  std::vector<std::string> fileList;
  std::vector<Eigen::VectorXd> fileAllData;

  Eigen::VectorXd upperJointsRefPos_with_hand = Eigen::VectorXd::Zero(adam_upper_actor_num + 12);
};

class StateStop : public FSMState
{
public:
  explicit StateStop(RobotData *robot_data);

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

private:
  Eigen::VectorXd init_joint_pos = Eigen::VectorXd::Zero(adam_upper_actor_num);
};
#endif
