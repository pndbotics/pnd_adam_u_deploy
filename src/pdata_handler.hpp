#ifndef PND_DATA_HANDLER_H_
#define PND_DATA_HANDLER_H_
#include "public_parament.h"

#include "../include/third_party/broccoli/core/Time.hpp"

#include "../include/third_party/spdlog/async.h"
#include "../include/third_party/spdlog/sinks/basic_file_sink.h"
#include "../include/third_party/spdlog/sinks/rotating_file_sink.h"
#include "../include/third_party/spdlog/sinks/stdout_color_sinks.h"
#include "../include/third_party/spdlog/spdlog.h"
#include <iostream>

class DataHandler {
public:
  static DataHandler &getInstance() {
    static DataHandler instance;
    return instance;
  }
  ~DataHandler() {
    logger_->flush();
    logger_retaregt_record->flush();
    spdlog::shutdown();
  }
  DataHandler(DataHandler const &) = delete;
  DataHandler &operator=(DataHandler const &) = delete;

  void init_log() {
    spdlog::init_thread_pool(8192, 1);
    std::filesystem::create_directories("logs");
    time_t currentTime = time(nullptr);
    char chCurrentTime[256];
    strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d_%H%M%S",
             localtime(&currentTime));
    std::string stCurrentTime = chCurrentTime;
    std::string filename = "logs/" + stCurrentTime + "log.txt";

    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        filename, 1024 * 1024 * 100, 3);
    rotating_sink->set_pattern("%v");
    std::vector<spdlog::sink_ptr> sinks{rotating_sink};
    logger_ = std::make_shared<spdlog::async_logger>(
        "loggername", sinks.begin(), sinks.end(), spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
    spdlog::register_logger(logger_);
  }

  void init_retarget_record() {
    std::filesystem::create_directories("resource");

    if (spdlog::get("retargetloggername")) {
      spdlog::drop("retargetloggername");
    }

    std::string retarget_mocap_file = "resource/retarget_record.txt";
    auto rotating_sink_retarget =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            retarget_mocap_file, 1024 * 1024 * 100, 0, rewrite_flag_);
    rotating_sink_retarget->set_pattern("%v");

    std::vector<spdlog::sink_ptr> sinks_retarget{rotating_sink_retarget};
    logger_retaregt_record = std::make_shared<spdlog::async_logger>(
        "retargetloggername", sinks_retarget.begin(), sinks_retarget.end(),
        spdlog::thread_pool(), spdlog::async_overflow_policy::block);
    spdlog::register_logger(logger_retaregt_record);
  }

  void cacheData(RobotData &robot_data, double timeFSM) {
    robot_data.dataL(0) = timeFSM;
    robot_data.dataL.segment(1, 25) = robot_data.q_a_;
    robot_data.dataL.segment(26, 25) = robot_data.q_dot_a_;
    robot_data.dataL.segment(51, 25) = robot_data.tau_a_;
    robot_data.dataL.segment(76, 25) = robot_data.q_d_;
    robot_data.dataL.segment(101, 25) = robot_data.q_dot_d_;
    robot_data.dataL.segment(126, 25) = robot_data.tau_d_;
  }

  void writeData(const RobotData &robot_data) {
    std::ostringstream oss;
    for (const auto &elem : robot_data.dataL) {
      oss << elem << " ";
    }
    logger_->info(oss.str());
  }

  void retraget_record_cacheData(RobotData &robot_data, double timeFSM) {
    robot_data.retarget_dataL.segment(0, 12) = robot_data.q_d_.segment(6, 12);
    robot_data.retarget_dataL.segment(12, 6) = robot_data.q_d_hands.head(6);
    robot_data.retarget_dataL.segment(18, 7) = robot_data.q_d_.tail(7);
    robot_data.retarget_dataL.segment(25, 6) = robot_data.q_d_hands.tail(6);
  }

  void retraget_record_writeData(const RobotData& robot_data) {
    static int count = 0;
    static bool file_format_fix = false;

    if (robot_data.retarget_record && rewrite_flag_) {
      init_retarget_record();
      rewrite_flag_ = false;
      count = 0;
      std::cout << "\033[1;32m" << "rewrite retarget log" << "\033[0m" << std::endl;
    }
    
    std::ostringstream oss_retaregt_record_;
    if(robot_data.retarget_record) {
      file_format_fix = true;
      count++;
      if(count % 6 == 0) {
        for (const auto& elem : robot_data.retarget_dataL) {
          oss_retaregt_record_ << elem << " ";
        }
        logger_retaregt_record->info(oss_retaregt_record_.str());
        oss_retaregt_record_.str("");
      }
    } else {
      rewrite_flag_ = true;
      if (file_format_fix) {
        trim_retarget_log();
        file_format_fix = false;
      }
    }
  }
private:
  void trim_retarget_log() {
    std::string log_path = "/home/pnd-humanoid/Documents/adam_u_demo/src/cpp_demo/resource/retarget_record.txt";
    std::ifstream in(log_path);
    std::vector<std::string> lines;
    std::string line;

    while (std::getline(in, line)) {
      lines.push_back(line);
    }
    in.close();

    if (!lines.empty()) {
      lines.erase(lines.begin());
      if (!lines.empty()) {
        lines.pop_back();
      }
    }

    std::ofstream out(log_path, std::ios::trunc);
    for (const auto& l : lines) {
      out << l << '\n';
    }
    out.close();
  }

private:
  DataHandler() = default;

private:
  std::shared_ptr<spdlog::async_logger> logger_;
  std::shared_ptr<spdlog::async_logger> logger_retaregt_record;
  bool rewrite_flag_ = true;

  int start_pos_ = 50;
};

#endif
