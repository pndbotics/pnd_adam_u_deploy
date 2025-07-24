#ifndef RETARGET_API_H_
#define RETARGET_API_H_

#include <string>

class StateRetarget {
 public:
  StateRetarget() = default;
  virtual ~StateRetarget() = default;

 public:
  virtual std::string get_data() = 0;

  virtual void set_adam_type(int type) = 0;

  virtual void set_flag(bool flag) = 0;
  virtual bool get_flag() = 0;

  virtual void set_post_handle(bool flag) = 0;
  virtual bool get_post_handle() = 0;

  virtual void set_post_handle_timer(int timer) = 0;
  virtual int get_post_handle_timer() = 0;

  virtual void set_tar_root_height(double root_height) = 0;
  virtual double get_tar_root_height() = 0;
  virtual void set_tar_root_height_last(double root_height) = 0;

  virtual double get_tar_root_height_last() = 0;
  virtual void map_tar_root_height(double root_height) = 0;

  virtual void subscribe() = 0;
  virtual void cancelSubscribe() = 0;
  virtual void exit() = 0;
};

extern "C" {
StateRetarget* getPndRetarget();
}
#endif  // RETARGET_API_H_