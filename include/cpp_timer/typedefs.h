#ifndef TIMER_TYPEDEFS_HH_
#define TIMER_TYPEDEFS_HH_

#include <map>
#include <string>
#include <chrono>
#include <memory>
#include <string_view>

namespace cpp_timer{

// Forward declarations
struct Layer;
class Ticker;

// Typedefs for readability
typedef std::shared_ptr<cpp_timer::Layer> LayerPtr;
typedef std::weak_ptr<cpp_timer::Layer> ParentPtr;
typedef std::chrono::steady_clock::time_point chronoTime;
typedef std::chrono::duration<std::chrono::steady_clock::rep, std::nano> chronoDuration;
typedef std::map<std::string_view, chronoDuration> durationMap;
typedef std::map<std::string_view, LayerPtr> layerMap;
typedef std::map<std::string_view, std::pair<int, chronoDuration>> timerTotal;

}

#endif