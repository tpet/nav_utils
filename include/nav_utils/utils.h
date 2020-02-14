#ifndef NAV_UTILS_UTILS_H
#define NAV_UTILS_UTILS_H

#include <thread>

namespace nav_utils
{

inline void sleepFor(const double secs)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(secs));
}

}

#endif //NAV_UTILS_UTILS_H
