#ifndef TIMER_TYPEDEFS_HH_
#define TIMER_TYPEDEFS_HH_

#include <map>
#include <string>
#include <chrono>
#include <memory>

namespace cpp_timer{

// Forward declarations
struct Layer;
class Ticker;

// Typedefs for readability
typedef std::shared_ptr<cpp_timer::Layer> LayerPtr;
typedef std::chrono::steady_clock::time_point chronoTime;
typedef std::chrono::duration<long int, std::nano> chronoDuration;
typedef std::map<std::string, chronoDuration> durationMap;
typedef std::map<std::string, LayerPtr> layerMap;
typedef std::map<std::string, std::pair<int, chronoDuration>> timerTotal;

}

#endif