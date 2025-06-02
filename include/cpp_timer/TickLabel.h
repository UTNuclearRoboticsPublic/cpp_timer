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
    std::thread::id thread_id;

    TicLabel(const char* n, chronoTime s, TickType t, std::thread::id id) : name(n), stamp(s), type(t), thread_id(id) {}
};

static inline std::ostream& operator<<(std::ostream& stream, const TicLabel& label) {
    stream << "{\n"
           << "\tName:   " << label.name << "\n"
           << "\tStamp:  " << label.stamp.time_since_epoch().count() << "\n"
           << "\tType:   " << (label.type == label.TICK ? "Tic" : "Toc") << "\n"
           << "\tThread: " << label.thread_id << "\n" 
           << "}";
    return stream;
} 

} // end namespace cpp_timer

#endif 
