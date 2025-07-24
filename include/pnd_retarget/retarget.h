#ifndef RETARGET_ROS_H_
#define RETARGET_ROS_H_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "retarget_api.h"

class JointStateSubscriber;

class StateRetargetImpl : public StateRetarget {
 public:
  StateRetargetImpl();
  virtual ~StateRetargetImpl();

 public:
  std::string get_data() override;
  void set_adam_type(int type) override;
  void set_flag(bool flag) override;
  bool get_flag() override;
  void set_post_handle(bool flag) override;
  bool get_post_handle() override;
  void set_post_handle_timer(int timer) override;
  int get_post_handle_timer() override;
  void set_tar_root_height(double root_height) override;
  double get_tar_root_height() override;
  void set_tar_root_height_last(double root_height) override;
  double get_tar_root_height_last() override;
  void map_tar_root_height(double root_height) override;
  void subscribe() override;
  void cancelSubscribe() override;
  void exit() override;

 private:
  double convert_radian_to_range(double radian);

  double convert_thumb_radian_to_range(double radian);
  void handle_inspire(const std::vector<double>& d);

  void ros_thread();

 private:
  std::shared_ptr<std::thread> ros_thread_;
  std::atomic<bool> shutdown_flag{false};
  std::shared_ptr<JointStateSubscriber> joint_state_subscriber_;
  bool motion_turn_on_ = false;
  bool post_handle_ = false;
  std::vector<double> mpc_dofs_pos_;
  std::vector<std::string> joint_name_;
  int adam_type_ = 0;
  int post_handle_timer_ = 0;
  int dof_pass_num_ = 0;
  double tar_root_height = 0.0;
  double tar_root_height_last = 0.0;
};

#endif  // RETARGET_ROS_H_