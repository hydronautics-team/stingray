#ifndef SRC_STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_COMMON_ASYNCTIMER_H_
#define SRC_STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_COMMON_ASYNCTIMER_H_


class AsyncTimer {

 private:

  bool busy = false;
  unsigned long milliseconds;

 public:

  AsyncTimer(unsigned long milliseconds);
  ~AsyncTimer() = default;

  bool start();
  bool isBusy();

};


#endif //SRC_STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_COMMON_ASYNCTIMER_H_
