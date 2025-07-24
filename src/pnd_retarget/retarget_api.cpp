
#include "retarget_api.h"


#include "retarget.h"

extern "C" {
StateRetarget* getPndRetarget() {
  static StateRetargetImpl instance;
  return &instance;
}
}
