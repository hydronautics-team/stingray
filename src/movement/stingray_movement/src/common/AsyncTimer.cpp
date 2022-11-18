#include "common/AsyncTimer.h"

#include <thread>

AsyncTimer::AsyncTimer(unsigned long milliseconds) : milliseconds(milliseconds) {}

bool AsyncTimer::start()
{
    if (busy)
    {
        return false;
    }

    busy = true;
    std::thread timerThread([=]()
                            {
    std::this_thread::sleep_for(std::chrono::milliseconds(this->milliseconds));
    busy = false; });
    timerThread.detach();

    return true;
}

bool AsyncTimer::isBusy()
{
    return busy;
}