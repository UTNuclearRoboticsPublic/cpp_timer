#ifndef TIMER_TICKER_H_
#define TIMER_TICKER_H_

namespace cpp_timer{

// Forward declaration
class Timer;

// For scope based measuring
class Ticker{
public:
    Ticker(const char* name, Timer* timer);
    ~Ticker();

private:
    Timer* timer_;
    const char* name_;
};

} // end namespace cpp_timer

#endif
