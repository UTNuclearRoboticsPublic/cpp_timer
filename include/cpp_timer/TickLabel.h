#ifndef TIMER_TICK_LABEL_HH_
#define TIMER_TICK_LABEL_HH_

#include "cpp_timer/typedefs.h"

namespace cpp_timer{

struct TicLabel{
    const char* name;
    chronoTime stamp;
    enum TickType{
        TICK,
        TOCK
    } type = TICK;

    TicLabel(const char* n, chronoTime s, TickType t) : name(n), stamp(s), type(t) {}
};

} // end namespace cpp_timer

#endif 
