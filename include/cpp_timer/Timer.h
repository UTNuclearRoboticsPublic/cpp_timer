#ifndef COVERAGE_PLANNER_TIMER_H_
#define COVERAGE_PLANNER_TIMER_H_

#include <map>
#include <math.h>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <assert.h>
#include <iostream>

namespace cpp_timer{

// Forward declaration
struct Layer;

typedef std::shared_ptr<cpp_timer::Layer> LayerPtr;
typedef std::chrono::high_resolution_clock::time_point chronoTime;
typedef std::map<std::string, double> durationMap;

struct Layer{
    int layer_index;
    std::map<std::string, long int> durations;
    std::map<std::string, LayerPtr> children;
    LayerPtr parent;
};

class Timer{
public:
    /**
     * Basic constructor, handles initialization
     */
    Timer();

    /**
     * Start timer for function
     * @brief                   Start timer for function
     * @param function_name     Name with which to store the function time
     *                          Does not have to match the actual function name
     * @return                  None
     * @note                    All tic() calls MUST be paired with a corresponding toc() call
     */
    void tic(std::string function_name);

    /**
     * Close timer for function
     * @brief                   Close timer for function
     * @param function_name     Name with which to store the function time
     *                          Does not have to match the actual function name
     * @return                  Duration of function call in microseconds
     */
    long int toc(std::string function_name);

    /**
     * Show the summary of the all of the timer calls. All tic() calls must be closed
     * when this function is called. Two versions of the summary will be printed. The 
     * first is a nested of view of function runtime, and the second is the total time
     * spent in each function, including time spent in nested functions
     * @brief                   Show the summary of all timer measurements
     */
    void summary();

private:
    /**
     * Pointer to the current layer
     */
    LayerPtr current_layer_;

    /**
     * Vector to function start times. After a function is completed, it's start time is removed
     */
    std::vector<chronoTime> start_times_;

    /**
     * Recursively print a layer and all of it's children
     */
    void printLayer_(const LayerPtr& layer, int prev_duration = 0);

    /**
     * Get the total time spent in a layer through recursive calculation
     */
    std::map<std::string, double> getTotals_(LayerPtr layer);
};

}   // namespace cpp_timer

#endif
