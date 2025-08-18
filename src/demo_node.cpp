#include "../include/pnd_adam_u_deploy.h"
#include "nlohmann/json.hpp"
#include "pdata_handler.hpp"
#include "pndrobotros2/msg/imu.hpp"
#include "pndrobotros2/msg/joint_state_cmd.hpp"
#include "pndrobotros2/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "retarget_api.h"
#include "robot_handler.h"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <linux/sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using namespace YAML;

void robot_state_callback(const pndrobotros2::msg::RobotState::SharedPtr msg,
                          RobotData &robot_data) {
  // update robot_data
  for (int i = 0; i < 25; i++) {
    robot_data.q_a_[i] = msg->q_a[i];
    robot_data.q_dot_a_[i] = msg->q_dot_a[i];
    robot_data.tau_a_[i] = msg->tau_a[i];
  }
  if (adam_type == ADAM_TYPE::Adam_U) {
    for (int i = 0; i < 12; i++) {
      robot_data.q_a_hands[i] = msg->hands_a[i];
    }
  }
  if (robot_data.get_actual_robot_state_flag) {
    robot_data.get_actual_robot_state_flag = false;
    for (int i = 0; i < 25; i++) {
      robot_data.q_d_[i] = robot_data.q_a_[i];
      robot_data.q_dot_d_[i] = robot_data.q_dot_a_[i];
      robot_data.tau_d_[i] = robot_data.tau_a_[i];
    }
    if (adam_type == ADAM_TYPE::Adam_U) {
      for (int i = 0; i < 12; i++) {
        robot_data.q_d_hands[i] = robot_data.q_a_hands[i];
      }
    }
  }
}

void commands_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("commands_subscriber"), "Received command: %s",
              msg->data.c_str());
  std::cout << "Received command: " << msg->data.c_str() << std::endl;
}

int main(int argc, char **argv) {
  mlockall(MCL_CURRENT | MCL_FUTURE);

  struct sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO, &param);

  rclcpp::init(argc, argv);

  double time_fsm = 0.0;
  DataHandler::getInstance().init_log();

  RobotData robot_data;
  Framework framework(robot_data);
  bool res = framework.init();
  if (!res) {
    std::cout << "framework init failed" << std::endl;
    return -1;
  }
  std::cout << "adam teleop " << getPndRetarget()->get_flag() << std::endl;
  getPndRetarget()->set_adam_type(adam_type);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("publisher_node", options);

  std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("pnd_adam_u_deploy");
  std::string param_file = package_share_dir + "/config/paras.yaml";
  try {
    Node config = LoadFile(param_file);

    std::function<void(Node, std::string)> parse_node;
    parse_node = [&](Node yaml_node, std::string prefix) {
      for (const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it) {
        std::string full_name =
            prefix.empty() ? it->first.as<std::string>()
                           : prefix + "." + it->first.as<std::string>();

        if (it->second.IsMap()) {
          parse_node(it->second, full_name);
        } else {
          auto param_value = it->second;
          if (param_value.IsScalar()) {
            try {
              double value = param_value.as<double>();
              std::cout << "Parameter: " << full_name << " = " << value
                        << std::endl;
            } catch (...) {
            }
          }
        }
      }
    };

    parse_node(config, "");

  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(node->get_logger(), "YAML parsing failed: %s", e.what());
    return -1;
  }

  auto publisher = node->create_publisher<pndrobotros2::msg::JointStateCmd>(
      "joint_cmd", rclcpp::QoS(10).reliable());

  auto robot_state_subscriber =
      node->create_subscription<pndrobotros2::msg::RobotState>(
          "robot_state", rclcpp::QoS(10).reliable(),
          [&robot_data](const pndrobotros2::msg::RobotState::SharedPtr msg) {
            robot_state_callback(msg, robot_data);
          });

  auto commands_subscriber = node->create_subscription<std_msgs::msg::String>(
      "/user_commands", rclcpp::QoS(10).reliable(), &commands_callback);

  pndrobotros2::msg::JointStateCmd joint_cmd;
  joint_cmd.q_d.resize(25);
  joint_cmd.q_dot_d.resize(25);
  joint_cmd.tau_d.resize(25);
  if (adam_type == ADAM_TYPE::Adam_U) {
    joint_cmd.hands_d.resize(12);
  }

  // 400Hz(2.5ms)
  auto timer = node->create_wall_timer(std::chrono::microseconds(2500), [&]() {
    /* USER CODE BEGIN 0 */
    for (int i = 0; i < 25; i++) {
      joint_cmd.q_d[i] = robot_data.q_d_[i];
      joint_cmd.q_dot_d[i] = 0;
      joint_cmd.tau_d[i] = 0;
    }
    if (adam_type == ADAM_TYPE::Adam_U) {
      for (int i = 0; i < 12; i++) {
        joint_cmd.hands_d[i] = robot_data.q_d_hands[i];
      }
    }
    publisher->publish(joint_cmd);
    /* USER CODE END 0 */

    /* USER CODE BEGIN 1 */
    if (!getPndRetarget()->get_flag() &&
        JsHum::getInst().get_real_time_motion_state() > 0) {
      getPndRetarget()->set_flag(true);
      getPndRetarget()->subscribe();
      std::cout << "real time retarget start" << std::endl;
    } else if (getPndRetarget()->get_flag() &&
               !getPndRetarget()->get_post_handle() &&
               JsHum::getInst().get_real_time_motion_state() < 0) {
      getPndRetarget()->cancelSubscribe();
      getPndRetarget()->set_post_handle(true);
      getPndRetarget()->set_post_handle_timer(1000);
      std::cout << "real time retarget post handle" << std::endl;
    }
    /* USER CODE END 1 */

    /* USER CODE BEGIN 2*/
    framework.runFSM(); // FSM
    /* USER CODE END 2*/

    /* USER CODE BEGIN 3*/
    time_fsm += kDt;
    DataHandler::getInstance().cacheData(robot_data, time_fsm);
    DataHandler::getInstance().writeData(robot_data);
    // retarget data record
    DataHandler::getInstance().retraget_record_cacheData(robot_data, time_fsm);
    DataHandler::getInstance().retraget_record_writeData(robot_data);
    /* USER CODE END 3*/

    /* USER CODE BEGIN 4*/
    // if(JsHum::getInst().disableJoints()){
    //     timer->cancel();
    //     rclcpp::shutdown();
    //     break;
    // }
    /* USER CODE END 4*/
  });

  rclcpp::spin(node);

  rclcpp::shutdown();

  getPndRetarget()->exit();
  std::cout << "retarget end!!!" << std::endl;

  return 0;
}
