#include "pnd_adam_u_deploy.h"
#include <iostream>
#include <unistd.h>

namespace cpp_lib_demo {

DemoLib::DemoLib(int init_value) : count_(init_value) {}

int DemoLib::getValue() const { return count_; }

void DemoLib::setValue(int new_value) { count_ = new_value; }

void DemoLib::increment() { count_++; }

int DemoLib::get_count() const { return count_; }

} // namespace cpp_lib_demo
