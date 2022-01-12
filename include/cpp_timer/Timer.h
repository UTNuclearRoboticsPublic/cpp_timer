///////////////////////////////////////////////////////////////////////////////
//      Title     : Timer.h
//      Project   :	cpp_timer
//      Author    : Alex Navarro
//      Created   : 05/25/2021
//      Copyright : Copyright© The University of Texas at Austin, 2014-2021. All rights reserved.
//                
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////


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
#include "boost/current_function.hpp"

// You need to define TIMER_INSTANCE in your own implementation file
#define TIMER_TIC TIMER_INSTANCE.tic(BOOST_CURRENT_FUNCTION)
#define TIMER_TOC TIMER_INSTANCE.toc(BOOST_CURRENT_FUNCTION)
#define TIMER_PTIC TIMER_INSTANCE->tic(BOOST_CURRENT_FUNCTION)
#define TIMER_PTOC TIMER_INSTANCE->toc(BOOST_CURRENT_FUNCTION)

namespace cpp_timer{

// Forward declaration
struct Layer;

// Typedefs for readability
typedef std::shared_ptr<cpp_timer::Layer> LayerPtr;
typedef std::chrono::steady_clock::time_point chronoTime;
typedef std::chrono::duration<long int, std::nano> chronoDuration;
typedef std::map<std::string, chronoDuration> durationMap;
typedef std::map<std::string, LayerPtr> layerMap;
typedef std::map<std::string, std::pair<int, chronoDuration>> timerTotal;

// TODO: Add call time to either Child or Layer and measure at runtime

struct Layer{
    std::string name;
    int layer_index;
    int call_count = 1;
    LayerPtr parent;
    layerMap children;
    chronoDuration duration = chronoDuration(0);
    chronoDuration child_tic_duration = chronoDuration(0);
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
     * @return                  None
     */
    void toc(std::string function_name);

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
     * Vector of all layers in the problem
     */
    std::vector<LayerPtr> all_layers_;

    /**
     * Recursively print a layer and all of it's children
     */
    void printLayer_(const LayerPtr& layer, long long prev_duration = 0);

    /**
     * Get the total time spent in a layer through recursive calculation
     */
    timerTotal getTotals_(LayerPtr layer);

    /**
     * Get the total times tic-toc pairs were called in children to a layer
     */
    chronoDuration getChildTicTocTime_(LayerPtr layer);
};

}   // namespace cpp_timer

#endif
