#include "cpp_timer/Timer.h"
#include "cpp_timer/Ticker.h"

namespace cpp_timer{

Ticker::Ticker(const char* name, Timer* timer) : name_(name), timer_(timer) {
    timer_->tic(name_);
}

Ticker::~Ticker(){
    timer_->toc(name_);
}

} // end namespace cpp_timer