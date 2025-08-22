
#include "retarget.h"

#include <signal.h>

#include <cmath>
#include <csignal>
#include <string>
#include <vector>

#include "json.hpp"
#include "nlohmann/json.hpp"
#include "public_parament.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

std::vector<std::string> pnd_hand_names_{
    "dof_pos/hand_pinky_Left",  "dof_pos/hand_ring_Left",     "dof_pos/hand_middle_Left",
    "dof_pos/hand_index_Left",  "dof_pos/hand_thumb_1_Left",  "dof_pos/hand_thumb_2_Left",
    "dof_pos/hand_pinky_Right", "dof_pos/hand_ring_Right",    "dof_pos/hand_middle_Right",
    "dof_pos/hand_index_Right", "dof_pos/hand_thumb_1_Right", "dof_pos/hand_thumb_2_Right"};

class JointStateSubscriber : public rclcpp::Node {
public:
  JointStateSubscriber(const std::vector<std::string> &joint_names,
                       const rclcpp::NodeOptions &options)
      : Node("adam_joint_state_subscriber", options) { // 添加options参数
    joint_position_.resize(joint_names.size(), 0);
    std::cout << "joint_names size: " << joint_names.size() << std::endl;

    for (const auto& hand_name : pnd_hand_names_) {
      auto it = std::find(joint_names.begin(), joint_names.end(), hand_name);
      if (it != joint_names.end()) {
        int hand_pos = std::distance(joint_names.begin(), it);
        joint_position_[hand_pos] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
      joint_idx_.insert(std::make_pair(joint_names[i], i));
    }
  }

  void subscribe() {
    std::cout << "subscription begin: " << subscription_ << std::endl;
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JointStateSubscriber::topic_callback, this, _1));
    std::cout << "subscription: " << subscription_ << std::endl;
  }
  void cancelSubscribe() { subscription_.reset(); }
  const std::vector<double> &get_data() const { return joint_position_; }
  const bool is_linear_actuator() {
    for (const std::string& hand_name : pnd_hand_names_) {
      if (joint_idx_.find(hand_name) != joint_idx_.end()) {
        if (!std::isnan(joint_position_[joint_idx_[hand_name]])) {
          return true;
        }
      }
    }
    return false;
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // std::cout << "Received JointState message:" << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Received JointState message:");
    // RCLCPP_INFO(this->get_logger(), "  Header stamp: %d.%d",
    // msg->header.stamp.sec, msg->header.stamp.nanosec);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      // std::cout << "  Joint: " << msg->name[i].c_str() << std::endl;
      // std::cout << "    Position: " << msg->position[i] << std::endl;
      // RCLCPP_INFO(this->get_logger(), "  Joint: %s", msg->name[i].c_str());
      // RCLCPP_INFO(this->get_logger(), "    Position: %.2f",
      // msg->position[i]);
      //   RCLCPP_INFO(this->get_logger(), "    Velocity: %.2f",
      //   msg->velocity[i]); RCLCPP_INFO(this->get_logger(), "    Effort:
      //   %.2f", msg->effort[i]);

      if (joint_idx_.find(msg->name[i]) != joint_idx_.end()) {
        joint_position_[joint_idx_[msg->name[i]]] = msg->position[i];
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  std::vector<double> joint_position_;
  std::map<std::string, int> joint_idx_;
};

StateRetargetImpl::StateRetargetImpl() {
  // setenv("LD_LIBRARY_PATH",
  //        "/opt/ros/humble/opt/yaml_cpp_vendor/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/"
  //        "x86_64-linux-gnu:/opt/ros/humble/lib",
  //        1);
  ros_thread_ =
      std::make_shared<std::thread>(&StateRetargetImpl::ros_thread, this);
  std::cout << "init ros2 subscriber thread:" << ros_thread_ << std::endl;
}

StateRetargetImpl::~StateRetargetImpl() {
  std::cout << "StateRetargetImpl destruct" << std::endl;
}

std::string StateRetargetImpl::get_data() {
  auto d = joint_state_subscriber_->get_data();

  for (size_t i = 0; i < dof_pass_num_; ++i) {
    mpc_dofs_pos_[i] = d[i];
  }
  if (adam_type_ == ADAM_TYPE::Adam_U || adam_type_ == ADAM_TYPE::Adam_U_handless) {
    if (joint_state_subscriber_->is_linear_actuator()) {
      int linear_actuator_idx = d.size() - pnd_hand_names_.size();
      for (size_t i = dof_pass_num_; i < mpc_dofs_pos_.size(); ++i) {
        mpc_dofs_pos_[i] = d[linear_actuator_idx + i - dof_pass_num_];
      }
    } else {
      handle_inspire(d);
    }
  }
  nlohmann::json j;
  j["data"] = mpc_dofs_pos_;
  return j.dump();
}

void StateRetargetImpl::set_adam_type(int type) {
  adam_type_ = type;
  if (adam_type_ == ADAM_TYPE::Adam_U || adam_type_ == ADAM_TYPE::Adam_U_handless) {
    std::cout << "adam type: AdamLite" << std::endl;
    joint_name_ = std::vector<std::string>{"dof_pos/waistRoll",
                                           "dof_pos/waistPitch",
                                           "dof_pos/waistYaw",
                                           "dof_pos/neckYaw",
                                           "dof_pos/neckPitch",

                                           "dof_pos/shoulderPitch_Left",
                                           "dof_pos/shoulderRoll_Left",
                                           "dof_pos/shoulderYaw_Left",
                                           "dof_pos/elbow_Left",
                                           "dof_pos/wristYaw_Left",
                                           "dof_pos/wristPitch_Left",
                                           "dof_pos/wristRoll_Left",

                                           "dof_pos/shoulderPitch_Right",
                                           "dof_pos/shoulderRoll_Right",
                                           "dof_pos/shoulderYaw_Right",
                                           "dof_pos/elbow_Right",
                                           "dof_pos/wristYaw_Right",
                                           "dof_pos/wristPitch_Right",
                                           "dof_pos/wristRoll_Right",

                                           "dof_pos/L_pinky_MCP_joint",
                                           "dof_pos/L_pinky_DIP_joint",
                                           "dof_pos/L_ring_MCP_joint",
                                           "dof_pos/L_ring_DIP_joint",
                                           "dof_pos/L_middle_MCP_joint",
                                           "dof_pos/L_middle_DIP_joint",
                                           "dof_pos/L_index_MCP_joint",
                                           "dof_pos/L_index_DIP_joint",
                                           "dof_pos/L_thumb_proximal",
                                           "dof_pos/L_thumb_PIP_joint",
                                           "dof_pos/L_thumb_MCP_joint1",

                                           "dof_pos/R_pinky_MCP_joint",
                                           "dof_pos/R_pinky_DIP_joint",
                                           "dof_pos/R_ring_MCP_joint",
                                           "dof_pos/R_ring_DIP_joint",
                                           "dof_pos/R_middle_MCP_joint",
                                           "dof_pos/R_middle_DIP_joint",
                                           "dof_pos/R_index_MCP_joint",
                                           "dof_pos/R_index_DIP_joint",
                                           "dof_pos/R_thumb_proximal",
                                           "dof_pos/R_thumb_PIP_joint",
                                           "dof_pos/R_thumb_MCP_joint1"};
    joint_name_.insert(joint_name_.end(), pnd_hand_names_.begin(), pnd_hand_names_.end());
    mpc_dofs_pos_.resize(31, 0);
    dof_pass_num_ = 19;
  } else {
    std::cout << "error adam type: " << adam_type_ << std::endl;
    std::exit(-1);
  }
}

void StateRetargetImpl::set_flag(bool flag) { motion_turn_on_ = flag; }

bool StateRetargetImpl::get_flag() { return motion_turn_on_; }

void StateRetargetImpl::set_post_handle(bool flag) { post_handle_ = flag; }

bool StateRetargetImpl::get_post_handle() { return post_handle_; }

void StateRetargetImpl::set_post_handle_timer(int timer) {
  post_handle_timer_ = timer;
}

int StateRetargetImpl::get_post_handle_timer() { return post_handle_timer_; }

void StateRetargetImpl::set_tar_root_height(double root_height) {
  tar_root_height = root_height;
}

double StateRetargetImpl::get_tar_root_height() { return tar_root_height; }

void StateRetargetImpl::set_tar_root_height_last(double root_height) {
  tar_root_height_last = root_height;
}

double StateRetargetImpl::get_tar_root_height_last() {
  return tar_root_height_last;
}

void StateRetargetImpl::map_tar_root_height(
    double root_height) { // [1.0, 0.6] map [0, -0.2]
  if (root_height > 1.0) {
    root_height = 1.0;
  } else if (root_height < 0.6) {
    root_height = 0.6;
  }
  tar_root_height = ((0 - (-0.2)) * (root_height - 0.6) / (1.0 - 0.6)) + (-0.2);
}

void StateRetargetImpl::subscribe() {
  std::cout << "ros2 subscriber " << std::endl;
  joint_state_subscriber_->subscribe();
}

void StateRetargetImpl::cancelSubscribe() {
  std::cout << "ros2 cancelSubscribe " << std::endl;
  joint_state_subscriber_->cancelSubscribe();
}

double StateRetargetImpl::convert_radian_to_range(double radian) {
  double mapped_value = 0.0;
  if (radian < 0)
    radian = 0;
  if (radian > M_PI)
    radian = M_PI;
  if (0 <= radian <= M_PI) {
    mapped_value = 1000 - int((radian / M_PI) * 1000);
    return mapped_value;
  } else
    std::cout << radian << "Input radians must be between 0 and pi" << std::endl;
}

double StateRetargetImpl::convert_thumb_radian_to_range(double radian) {
  // std::cout << "convert_thumb_radian_to_range: " << radian << std::endl;
  double mapped_value = 0.0;
  if (radian < 0)
    radian = 0;
  if (radian > 0.6)
    radian = 0.6;
  if (0 <= radian && radian <= 0.6) {
    mapped_value = 1000 - int((radian / 0.6) * 1000);
    return mapped_value;
  } else
    std::cout << radian << "Input radians must be between 0 and pi" << std::endl;
}

double convert_thumb_radian_to_range1(double radian) {
  // std::cout << "convert_thumb_radian_to_range1: " << radian << std::endl;
  double mapped_value = 0.0;
  if (radian < 0.1)
    radian = 0.1;
  if (radian > 0.3)
    radian = 0.3;
  if (0 <= radian && radian <= 0.3) {
    mapped_value = 1000 - int(((radian - 0.1) / (0.3 - 0.1)) * 1000);
    return mapped_value;
  } else
    std::cout << radian << "Input radians must be between 0 and pi" << std::endl;
}

void StateRetargetImpl::handle_inspire(const std::vector<double> &d) {
  // 打印 调试信息
  mpc_dofs_pos_[19] = (convert_radian_to_range(d[19] + d[20]));
  mpc_dofs_pos_[20] = (convert_radian_to_range(d[21] + d[22]));
  mpc_dofs_pos_[21] = (convert_radian_to_range(d[23] + d[24]));
  mpc_dofs_pos_[22] = (convert_radian_to_range(d[25] + d[26]));
  mpc_dofs_pos_[23] = (convert_thumb_radian_to_range1(d[27] + d[28]));
  mpc_dofs_pos_[24] = (convert_thumb_radian_to_range(d[29]));
  mpc_dofs_pos_[25] = (convert_radian_to_range(d[30] + d[31]));
  mpc_dofs_pos_[26] = (convert_radian_to_range(d[32] + d[33]));
  mpc_dofs_pos_[27] = (convert_radian_to_range(d[34] + d[35]));
  mpc_dofs_pos_[28] = (convert_radian_to_range(d[36] + d[37]));
  mpc_dofs_pos_[29] = (convert_thumb_radian_to_range1(d[38] + d[39]));
  mpc_dofs_pos_[30] = (convert_thumb_radian_to_range(d[40]));
}

void StateRetargetImpl::ros_thread() {
  std::cout << "joint_state_subscriber begin" << std::endl;

  auto context = rclcpp::contexts::get_global_default_context();
  rclcpp::NodeOptions options;
  options.context(context); // 共享上下文

  joint_state_subscriber_ =
      std::make_shared<JointStateSubscriber>(joint_name_, options);

  // joint_state_subscriber_ =
  // std::make_shared<JointStateSubscriber>(joint_name_);
  std::cout << "joint_state_subscriber_" << std::endl;

  rclcpp::Rate rate(200);
  while (rclcpp::ok() && !shutdown_flag) {
    rclcpp::spin_some(joint_state_subscriber_); // non-blocking
    rate.sleep();
  }
  std::cout << "retarget shutdown_flag" << std::endl;

  rclcpp::shutdown();
  std::cout << "rclcpp shutdown" << std::endl;
}

void StateRetargetImpl::exit() {
  std::cout << "StateRetargetImpl exit begin" << std::endl;
  shutdown_flag = true;
  std::cout << "set shutdown_flag true" << std::endl;
  ros_thread_->join();
  std::cout << "ros_thread exit" << std::endl;
}
