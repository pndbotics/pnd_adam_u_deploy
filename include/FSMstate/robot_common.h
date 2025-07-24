#ifndef ROBOT_COMMON_H_
#define ROBOT_COMMON_H_

#include <Eigen/Dense>
#include <vector>
#include "public_parament.h"

// 常量定义
constexpr double kDt = 0.0025;  // 控制周期(秒)

// Status code enumeration
enum class AdamStatusCode {
    UNKNOWN = -1,       // Unknown state
    SUCCESS = 0,        // Operation succeeded
    FAILURE = 1         // Operation failed
};

// Finite State Machine states enumeration
enum class FSMStateName {
    STOP = 0,           // Stop state
    ZERO,               // Zero position state
    TELEOPERATION       // Teleoperation control state
};
std::ostream& operator<<(std::ostream& os, FSMStateName state);

class RobotCommon {
public:
    virtual ~RobotCommon() = default;
    
    // Core interface methods
    virtual AdamStatusCode init() = 0;
    virtual AdamStatusCode getState() = 0;
    virtual AdamStatusCode setCommand() = 0;
    virtual AdamStatusCode disableAllJoints() = 0;

    Eigen::VectorXd joint_Kp_ = Eigen::VectorXd::Zero(17);
    Eigen::VectorXd joint_Kd_ = Eigen::VectorXd::Zero(17);

protected:
    Eigen::VectorXd joint_pos_ = Eigen::VectorXd::Zero(17);
    Eigen::VectorXd joint_vel_ = Eigen::VectorXd::Zero(17);
    Eigen::VectorXd joint_tau_ = Eigen::VectorXd::Zero(17);
};

class FSMState {
public:
    explicit FSMState(RobotData *robot_data) { robot_data_ = robot_data; };
    virtual ~FSMState() = default;

    virtual void onEnter() = 0;             
    virtual void run() = 0;                 
    virtual FSMStateName checkTransition() = 0; 
    virtual void onExit() = 0;              

    FSMStateName current_state_name_;       

protected:
    double timer_ = 0.;                     

    RobotData *robot_data_;
};

struct StateList {
    FSMState *zero;           
    FSMState *stop;           
    FSMState *teleoperation;  
};

#endif // ROBOT_COMMON_HPP_