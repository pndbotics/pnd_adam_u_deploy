#include "robot_common.h"
#include <ostream>

std::ostream& operator<<(std::ostream& os, FSMStateName state) {
    switch(state) {
        case FSMStateName::STOP: os << "STOP"; break;
        case FSMStateName::ZERO: os << "ZERO"; break;
        case FSMStateName::TELEOPERATION: os << "TELEOPERATION"; break;
        default: os << "UNKNOWN";
    }
    return os;
}
