#ifndef TIMER_TOTAL_HH_
#define TIMER_TOTAL_HH_

#include <string>
#include <chrono>
#include "cpp_timer/typedefs.h"

namespace cpp_timer{

struct TimerTotal{
    const char* name;
    int call_count = 0;
    chronoDuration duration = chronoDuration(0);

    TimerTotal() = default;
    TimerTotal(const char* name_, int call_count_, chronoDuration dur_) : name(name_), call_count(call_count_), duration(dur_) {}

    static bool compareTotalByName(TimerTotal P1, TimerTotal P2){
        return P1.name < P2.name;
    }

    static bool compareTotalByAverage(TimerTotal P1, TimerTotal P2){
        return (float)P1.duration.count()/(float)P1.call_count > (float)P2.duration.count()/(float)P2.call_count;
    }

    static bool compareTotalByTotal(TimerTotal P1, TimerTotal P2){
        return P1.duration > P2.duration;
    }

    static bool compareTotalByCallCount(TimerTotal P1, TimerTotal P2){
        return P1.call_count > P2.call_count;
    }
};

} // end namespace cpp_timer

#endif