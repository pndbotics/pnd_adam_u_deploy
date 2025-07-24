#ifndef CPP_LIB_DEMO__CPP_LIB_DEMO_HPP_
#define CPP_LIB_DEMO__CPP_LIB_DEMO_HPP_

namespace cpp_lib_demo {

class DemoLib {
 public:
  explicit DemoLib(int init_value = 0);
  int getValue() const;
  void setValue(int new_value);
  void increment();
  int get_count() const;

 private:
  int value_;
  int count_;
};

}  

#endif  
