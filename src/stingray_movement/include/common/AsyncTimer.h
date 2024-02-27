#ifndef SRC_STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_COMMON_ASYNCTIMER_H_
#define SRC_STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_COMMON_ASYNCTIMER_H_

#include <thread>

class AsyncTimer {

private:

  bool busy = false;
  unsigned long milliseconds;

public:

  AsyncTimer(unsigned long milliseconds) : milliseconds(milliseconds) {}
  ~AsyncTimer() = default;

  bool start() {
    if (busy) {
      return false;
    }

    busy = true;
    std::thread timerThread([=]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(this->milliseconds));
      busy = false;
      });
    timerThread.detach();

    return true;
  };

  bool isBusy() {
    return busy;
  };


#endif //SRC_STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_COMMON_ASYNCTIMER_H_
