
#include "fsm_state.h"
#include "iostream"

#include "json.hpp"
#include "retarget_api.h"

void FSMInit() { JsHum::getInst().init(); }

StateZero::StateZero(RobotData *robot_data) : FSMState(robot_data) {
  current_state_name_ = FSMStateName::ZERO;
}

void StateZero::onEnter() {
  timer_ = 0.;
  std::cout << "StateZero::onEnter()" << std::endl;

  init_joint_pos = robot_data_->q_a_.tail(adam_upper_actor_num);
  std::cout << "init_joint_pos: " << init_joint_pos.transpose() << std::endl;
}

void StateZero::run() {
  // std::cout << "StateZero::run()" << std::endl;
  if (timer_ < total_time) {
    fifthPoly(init_joint_pos, zero_, zero_, zero_, zero_, zero_, total_time,
              timer_, joint_pos_, joint_vel_, joint_acc_);
  } else {
    zero_finish_flag = true;
  }
  // std::cout << "joint_pos_: " << joint_pos_.transpose() << std::endl;
  robot_data_->q_d_.tail(adam_upper_actor_num) = joint_pos_;
  robot_data_->q_dot_d_.setZero();
  robot_data_->tau_d_.setZero();

  timer_ += kDt;
}

FSMStateName StateZero::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoTELEOPERATION" &&
      zero_finish_flag) {
    std::cout << "Zero2TELEOPERATION" << std::endl;
    return FSMStateName::TELEOPERATION;
  } else if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "Zero2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::ZERO;
  }
}

void StateZero::onExit() { zero_finish_flag = false; }

StateTeleoperation::StateTeleoperation(RobotData *robot_data)
    : FSMState(robot_data) {
  current_state_name_ = FSMStateName::TELEOPERATION;
}

StateTeleoperation::~StateTeleoperation() {}

void StateTeleoperation::onEnter() {
  std::cout << "StateTeleoperation::onEnter()" << std::endl;

  timer_ = 0.;
  motion_start_flag = true;
}

void StateTeleoperation::run() {
  retarget_teleoperation();
  retarget_motion_play();

  timer_ += kDt;
}

FSMStateName StateTeleoperation::checkTransition() {
  if (JsHum::getInst().get_motion_play_state()) {
    robot_data_->retarget_play_flag = true;
  }
  robot_data_->retarget_play_flag_continue =
      JsHum::getInst().get_motion_play_continue_state();

  robot_data_->retarget_record = JsHum::getInst().get_motion_record_state();

  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "TELEOPERATION2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::TELEOPERATION;
  }
}

void StateTeleoperation::onExit() {}

void StateTeleoperation::retarget_teleoperation() {
  static bool state_flag = true;
  static double timer_start = 0.0;
  static double ratio_scale = 0.0;
  if (getPndRetarget()->get_flag()) {
    if (getPndRetarget()->get_post_handle()) {
      state_flag = true;
      ratio_scale = 0.0;
      Eigen::VectorXd upperJointsRefVel =
          Eigen::VectorXd::Zero(adam_upper_actor_num);
      if (getPndRetarget()->get_post_handle_timer() > 0) {
        double ratio = 0.005;
        upperJointsRefPosFilter =
            (1 - ratio) * upperJointsRefPosLast + ratio * upperJointsCmd;
        double tar_height_ratio = 0.01;
        auto tmp =
            (1 - tar_height_ratio) * getPndRetarget()->get_tar_root_height() +
            tar_height_ratio * robot_data_->pCmd_joystick(2);
        getPndRetarget()->set_tar_root_height(tmp);
        robot_data_->q_d_.tail(adam_upper_actor_num) = upperJointsRefPosFilter;
        upper_joints_x_d.row(0).tail(adam_upper_actor_num) =
            robot_data_->q_d_.tail(adam_upper_actor_num);

        robot_data_->q_dot_d_.tail(adam_upper_actor_num) = upperJointsRefVel;
        upper_joints_x_d.row(1).tail(adam_upper_actor_num) =
            robot_data_->q_dot_d_.tail(adam_upper_actor_num);

        upperJointsRefPosLast = upperJointsRefPosFilter;
        getPndRetarget()->set_post_handle_timer(
            getPndRetarget()->get_post_handle_timer() - 1);
      } else {
        upperJointsRefPosFilter = upperJointsCmd;
        getPndRetarget()->set_post_handle(false);
        getPndRetarget()->set_flag(false);
        std::cout << "real time retarget end" << std::endl;
      }
    } else {
      Eigen::VectorXd upperJointsRefVel =
          Eigen::VectorXd::Zero(adam_upper_actor_num);
      auto data = getPndRetarget()->get_data();
      getPndRetarget()->publisher(robot_data_->q_a_.tail(adam_upper_actor_num), robot_data_->q_d_hands);
      try {
        auto data_j = nlohmann::json::parse(data);

        for (size_t i = 0; i < upperJointsRefPos.size(); ++i) {
          upperJointsRefPos(i) = data_j["data"][i];
        }
        if (adam_type == ADAM_TYPE::Adam_U) {
          robot_data_->q_d_hands.resize(12);
          for (size_t i = 0; i < robot_data_->q_d_hands.size(); ++i) {
            robot_data_->q_d_hands(i) =
                data_j["data"][i + upperJointsRefPos.size()];
          }
        }
        if (state_flag == true) {
          timer_start = timer_ + 1.0;
          state_flag = false;
        }
        if (timer_ < timer_start) {
          ratio_scale += 0.0025;
        } else {
          ratio_scale = 1.0;
        }
        double ratio = 0.035 * ratio_scale;
        upperJointsRefPosFilter =
            ratio * upperJointsRefPos + (1 - ratio) * upperJointsRefPosFilter;
      } catch (const nlohmann::json::parse_error &e) {
        std::cerr << e.what() << '\n';
      }

      robot_data_->q_d_.tail(adam_upper_actor_num) = upperJointsRefPosFilter;
      upper_joints_x_d.row(0).tail(adam_upper_actor_num) =
          robot_data_->q_d_.tail(adam_upper_actor_num);

      robot_data_->q_dot_d_.tail(adam_upper_actor_num) = upperJointsRefVel;
      upper_joints_x_d.row(1).tail(adam_upper_actor_num) =
          robot_data_->q_dot_d_.tail(adam_upper_actor_num);

      upperJointsRefPosLast = upperJointsRefPosFilter;
    }
  }
}

void StateTeleoperation::retarget_motion_play() {
  if (robot_data_->retarget_play_flag && timer_ > totaltime) {
    Eigen::VectorXd upperJointsRefVel =
        Eigen::VectorXd::Zero(adam_upper_actor_num);
    double motionDt = kDt * 6;
    static int motionNumber = 0;

    if (motion_start_flag) {
      /*Read file count and file sorting start*/

      std::string path;
      path =
          std::filesystem::current_path() /
          "install/pnd_adam_u_deploy/share/pnd_adam_u_deploy/resource/motion/";
      int fileCount = 0;
      if (robot_data_->motionselect == "") {
        DIR *dir = opendir(path.c_str()); // open directory
        struct dirent *entry;
        fileList.clear();
        while ((entry = readdir(dir)) != nullptr) { // get the target file name
          if (entry->d_type == DT_REG) {
            fileList.push_back(entry->d_name);
            fileCount++;
          }
        }
        closedir(dir);
        std::sort(fileList.begin(), fileList.end());

        if (robot_data_->motionNumber >= fileCount) {
          robot_data_->motionNumber = 0;
          std::cout << "\033[1;31m" << "motionNumber is out of range"
                    << "\033[0m" << std::endl;
        }
        robot_data_->motionPath =
            std::string(std::filesystem::current_path() /
                        "install/pnd_adam_u_deploy/share/pnd_adam_u_deploy/"
                        "resource/motion/") +
            fileList[robot_data_->motionNumber];

        motionNumber = fileCount;
      } else {
        std::string targetFile = robot_data_->motionselect;
        std::cout << robot_data_->motionselect << std::endl;
        std::cout << targetFile << std::endl;

        DIR *dir = opendir(path.c_str());
        struct dirent *entry;

        fileList.clear();
        while ((entry = readdir(dir)) !=
               nullptr) { // iterate through all files and subdirectories in the
                          // directory
          if (entry->d_type == DT_REG) {
            fileList.push_back(entry->d_name);
            fileCount++;
          }
        }
        closedir(dir);
        std::sort(fileList.begin(), fileList.end());

        // check the file number in fileList
        int targetFileIndex = -1;
        for (size_t i = 0; i < fileList.size(); ++i) {
          if (fileList[i] == targetFile) {
            targetFileIndex = i;
            break;
          }
        }
        if (targetFileIndex == -1) {
          std::cout << "\033[1;31m" << "motion file not found: " << targetFile
                    << "\033[0m" << std::endl;
          mocap_file_ok = false;
          robot_data_->retarget_play_flag = false;
          return;
        }
        robot_data_->motionNumber = targetFileIndex;

        robot_data_->motionPath =
            std::string(std::filesystem::current_path() /
                        "install/pnd_adam_u_deploy/share/pnd_adam_u_deploy/"
                        "resource/motion/") +
            fileList[robot_data_->motionNumber];

        motionNumber = fileCount;
      }

      std::cout << "\033[1;32m" << "motionPath:   " << robot_data_->motionPath
                << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "motionNumber: " << robot_data_->motionNumber + 1 << "/"
                << motionNumber << "\033[0m" << std::endl;
      /*Read file count and file sorting end*/

      /*read all line start*/
      infile.open(robot_data_->motionPath, std::ios::in);
      std::string line;
      fileAllData.clear();
      mocap_motion_line_num = 0;
      while (static_cast<bool>(getline(infile, line))) {
        if (infile.is_open()) {
          std::stringstream ss(line);
          int j = 0;
          while (ss >> upperJointsRefPos_with_hand(j)) {
            j++;
            if (j >= upperJointsRefPos_with_hand.size()) {
              break;
            }
          }
          // std::cout << "line: " << mocap_motion_line_num << " " <<
          // upperJointsRefPos_with_hand.transpose() << std::endl;
          fileAllData.push_back(upperJointsRefPos_with_hand);
          mocap_motion_line_num++;
        }
      }
      /*read all line end*/

      time1 = timer_;
      time2 = time1 + 1.0;
      time3 = time2 + motionDt * mocap_motion_line_num;
      time4 = time3 + 1.0;

      mocap_motion_line_num = 0;
      if (adam_type == ADAM_TYPE::Adam_U) {
        // get current joint position
        upperJointsCmd = robot_data_->q_a_.tail(adam_upper_actor_num);
        // set target joint position
        upperJointsRefPos.head(12) =
            fileAllData[mocap_motion_line_num].head(12);
        upperJointsRefPos.tail(7) =
            fileAllData[mocap_motion_line_num].segment(18, 7);
        // set target hand position
        robot_data_->q_d_hands.resize(12);
        robot_data_->q_d_hands
            << fileAllData[mocap_motion_line_num].segment(12, 6),
            fileAllData[mocap_motion_line_num].segment(25, 6);
      } else if (adam_type == ADAM_TYPE::Adam_U_handless) {
        // get current joint position
        upperJointsCmd = robot_data_->q_a_.tail(adam_upper_actor_num);
        // set target joint position
        upperJointsRefPos.head(12) =
            fileAllData[mocap_motion_line_num].head(12);
        upperJointsRefPos.tail(7) =
            fileAllData[mocap_motion_line_num].segment(18, 7);
      } else {
        std::cout << "open motion file failed!!!" << std::endl;
        mocap_file_ok = false;
        robot_data_->retarget_play_flag = false;
      }
      upperJointsRefPosLast = upperJointsRefPos;
      motion_start_flag = false;

      /* print */
      std::cout << "mocap_motion_line_num: " << mocap_motion_line_num
                << std::endl;
      std::cout << "time1: " << time1 << std::endl;
      std::cout << "time2: " << time2 << std::endl;
      std::cout << "time3: " << time3 << std::endl;
      std::cout << "time4: " << time4 << std::endl;
      infile.close();
    }
    /*play motion*/
    if (timer_ <= time2 && mocap_file_ok) {
      upperJointsRefPosFilter =
          (1.0 - (timer_ - time1) / (time2 - time1)) * upperJointsCmd +
          ((timer_ - time1) / (time2 - time1)) *
              upperJointsRefPos; // 从upperJointsCmd到upperJointsRefPos
    } else if (timer_ <= time3 && mocap_file_ok) {
      if ((int)((timer_ - time2) / kDt) % (int)(motionDt / kDt) == 0) {
        if (mocap_motion_line_num <= fileAllData.size() - 1) {
          if (adam_type == ADAM_TYPE::Adam_U) {
            // set target joint position
            upperJointsRefPos.head(12) =
                fileAllData[mocap_motion_line_num].head(12);
            upperJointsRefPos.tail(7) =
                fileAllData[mocap_motion_line_num].segment(18, 7);
            // set target hand position
            robot_data_->q_d_hands.resize(12);
            robot_data_->q_d_hands
                << fileAllData[mocap_motion_line_num].segment(12, 6),
                fileAllData[mocap_motion_line_num].segment(25, 6);
          } else if (adam_type == ADAM_TYPE::Adam_U_handless) {
            // set target joint position
            upperJointsRefPos.head(12) =
                fileAllData[mocap_motion_line_num].head(12);
            upperJointsRefPos.tail(7) =
                fileAllData[mocap_motion_line_num].segment(18, 7);
          } else {
            std::cout << "error adam type" << std::endl;
            return;
          }
          mocap_motion_line_num++;
        }
      }
      double ratio = 0.03;
      upperJointsRefPosFilter =
          ratio * upperJointsRefPos + (1 - ratio) * upperJointsRefPosFilter;
    } else if (timer_ <= time4 && mocap_file_ok) {
      upperJointsRefPosFilter =
          (1.0 - (timer_ - time3) / (time4 - time3)) * upperJointsRefPos +
          ((timer_ - time3) / (time4 - time3)) * upperJointsCmd;
      robot_data_->q_d_hands.resize(12);
      robot_data_->q_d_hands << 100, 100, 100, 100, 200, 1000, 100, 100, 100,
          100, 200, 1000;
    } else {
      std::cout << "motion end" << std::endl;
      upperJointsRefPosFilter = upperJointsCmd;
      if (!robot_data_->retarget_play_flag_continue) {
        robot_data_->retarget_play_flag = false;
      }
      motion_start_flag = true;
      robot_data_->motionNumber++;
      robot_data_->motionNumber %= motionNumber;
    }

    robot_data_->q_d_.tail(adam_upper_actor_num) = upperJointsRefPosFilter;
    robot_data_->q_dot_d_.tail(adam_upper_actor_num) = upperJointsRefVel;
    upperJointsRefPosLast = upperJointsRefPosFilter;
  }
}

void StateTeleoperation::retarget_motion_record() {}
StateStop::StateStop(RobotData *robot_data) : FSMState(robot_data) {
  current_state_name_ = FSMStateName::STOP;
}

void StateStop::onEnter() { timer_ = 0.; }

void StateStop::run() {
  robot_data_->q_d_.tail(adam_upper_actor_num - 2) =
      robot_data_->q_a_.tail(adam_upper_actor_num - 2);
  robot_data_->q_dot_d_.setZero();
  robot_data_->tau_d_.setZero();
  timer_ += kDt;
}

FSMStateName StateStop::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoZero") {
    std::cout << "Stop2Zero" << std::endl;
    return FSMStateName::ZERO;
  } else {
    return FSMStateName::STOP;
  }
}

void StateStop::onExit() {}
