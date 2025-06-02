///////////////////////////////////////////////////////////////////////////////
//      Title     : Timer.h
//      Project   :	cpp_timer
//      Author    : Alex Navarro
//      Created   : 05/25/2021
//      Copyright : Copyright© The University of Texas at Austin, 2014-2021. All rights reserved.
//                
//      Redistribution and use in source and binary forms, with or without modification,
//      are permitted provided that the following conditions are met:
//
//      Redistributions of source code must retain the above copyright notice, this
//      list of conditions and the following disclaimer.
//      
//      Redistributions in binary form must reproduce the above copyright notice, this
//      list of conditions and the following disclaimer in the documentation and/or
//      other materials provided with the distribution.
//
//      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//      ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//      DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//      ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//      (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//      LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//      ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//      SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef TIMER_H_
#define TIMER_H_

#include <map>
#include <mutex>
#include <atomic>
#include <math.h>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <condition_variable>
#include "boost/preprocessor/cat.hpp"
#include "boost/current_function.hpp"

#include "cpp_timer/Layer.h"
#include "cpp_timer/Ticker.h"
#include "cpp_timer/TickLabel.h"
#include "cpp_timer/TimerTotal.h"

// You need to define TIMER_INSTANCE in your own implementation file
#define TIMER_TIC TIMER_INSTANCE.tic(BOOST_CURRENT_FUNCTION)
#define TIMER_TOC TIMER_INSTANCE.toc(BOOST_CURRENT_FUNCTION)
#define TIMER_PTIC TIMER_INSTANCE->tic(BOOST_CURRENT_FUNCTION)
#define TIMER_PTOC TIMER_INSTANCE->toc(BOOST_CURRENT_FUNCTION)
#define STIC  const auto&& BOOST_PP_CAT(_ticker_, __LINE__) = TIMER_INSTANCE.scopedTic(BOOST_CURRENT_FUNCTION)
#define SPTIC const auto&& BOOST_PP_CAT(_ticker_, __LINE__) = TIMER_INSTANCE->scopedTic(BOOST_CURRENT_FUNCTION)

namespace cpp_timer{

class Timer{
public:
    /**
     * Basic constructor, handles initialization
     */
    Timer();

    /**
     * Close up loose threads on desctruction 
     */
    ~Timer();

    using time_t = std::chrono::steady_clock::rep;

    /**
     * Start timer for function
     * @param function_name     Name with which to store the function time
     *                          Does not have to match the actual function name
     * @return                  None
     * @note                    All tic() calls MUST be paired with a corresponding toc() call
     */
    void tic(const char* function_name);

    /**
     * Close timer for function
     * @param function_name     Name with which to store the function time
     *                          Does not have to match the actual function name
     * @return                  None
     */
    void toc(const char* function_name);

    /**
     * Start timer for function (CPU time)
     * @param function_name     Name with which to store the function time
     *                          Does not have to match the actual function name
     * @return                  None
     * @note                    All ticCpu() calls MUST be paired with a corresponding tocCpu() call
     */
    void ticCpu(const char* function_name);

    /**
     * Close timer for function (CPU time)
     * @param function_name     Name with which to store the function time
     *                          Does not have to match the actual function name
     * @return                  None
     */
    void tocCpu(const char* function_name);

    /**
     * Start a scope based timer for a scope 
     * @param name              The name of the scope or function to measure
     * @return                  A ticker scope object that automatically measures its own lifetime
     */
    Ticker scopedTic(const char* name);

    void forceTrigger() {
        std::unique_lock<std::mutex> tree_lock(tree_mtx_);
        force_trigger_ = true;
        tree_contition_.notify_one();
        tree_contition_.wait(tree_lock, [this]() -> bool {return !force_trigger_;});
    }

    /**
     * Start timing a multithreaded portion of code
     */
    void enterMultithreadedRegion() {
        forceTrigger();
        multithreaded_ = true;
    }

    /**
     * Conclude timing a multithreaded portion of code
     */
    void exitMultithreadedRegion() {
        multithreaded_ = false;
        last_batch_was_multithreaded_ = true;
        forceTrigger();
    }

    enum SummaryOrder{
        BY_NAME,
        BY_TOTAL,
        BY_AVERAGE,
        BY_CALL_COUNT,
        BY_CALL_ORDER
    };

    /**
     * Get the string version of the summary. Useful if you want to export to a file or print 
     * through some logging mechanism other than std::cout
     * @param total_order       The order in which to present the order of function calls in the overarching summary
     * @param breakdown_order   The order in whcih to present the order of function calls in the detailed summary
     */
    std::string get_summary_string(SummaryOrder total_order = BY_NAME, SummaryOrder breakdown_order = BY_CALL_ORDER);

    /**
     * Show the summary of the all of the timer calls. All tic() calls must be closed
     * when this function is called. Two versions of the summary will be printed. The 
     * first is a nested of view of function runtime, and the second is the total time
     * spent in each function, including time spent in nested functions
     * @brief                   Show the summary of all timer measurements
     */
    void summary(SummaryOrder total_order = BY_NAME, SummaryOrder breakdown_order = BY_CALL_ORDER);

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
     * Vectors which hold the start and end times of each function 
     */
    std::vector<TicLabel> tictocs_;

    /**
     * A map of all parsed function names against their original names. If a parsed name
     * occurs multiple times, then a namespace can be added on to help differentiate the function calls
     */
    std::multimap<std::string, const char*> function_name_map_;

    /**
     * Thread which works on building the tree while allowing the main thread to run uninterrupted 
     */
    std::thread tree_thread_;

    /**
     * Mutex to protect the tic and toc labels while they are being gathered 
     */
    std::mutex tree_mtx_;

    /**
     * Condition variable to tell tree thread when to build up the tree 
     */
    std::condition_variable tree_contition_;

    /**
     * Boolean flag indicating whether the thread should conclude 
     */
    volatile bool concluded_ = true;

    /**
     * Vector of all layers in the problem
     */
    std::vector<LayerPtr> all_layers_;

    /**
     * A stringstream object used to aggregate results for summary printing
     */
    std::stringstream summary_ss_;

    /**
     * A flag to force the tree to update. After the update is complete,
     * the flag automatically resets to false
     */
    std::atomic<bool> force_trigger_ = false;

    /**
     * Flag to indicate whether additional metadata should be collected in a multithreaded region
     */
    bool multithreaded_ = false;

    /**
     * Flag to indicate that the last batch of Tictoc labels was performed multithreaded
     */
    bool last_batch_was_multithreaded_ = false;

    /**
     * ID of the main thread in which the timer is created
     */
    std::thread::id main_thread_id_{std::this_thread::get_id()};

    /**
     * Get the CPU time point in terms of chrono time 
     */
    chronoTime getCPUTimePoint_() const;

    /**
     * Recursively print a layer and all of it's children
     */
    void printLayer_(const LayerPtr& layer, SummaryOrder order, time_t prev_duration = 0);

    /**
     * Get the total time spent in a layer through recursive calculation
     */
    // timerTotal getTotals_(LayerPtr layer);
    void getTotals_(LayerPtr layer);

    /**
     * Get the total times tic-toc pairs were called in children to a layer
     */
    chronoDuration getChildTicTocTime_(LayerPtr layer);

    /**
     * Insert a tic or toc label into the tree 
     */
    void treeTic_(TicLabel);
    void treeToc_(TicLabel);

    /**
     * Tree thread loop function, which retreives all the current tics and tocs and builds up 
     * the tree as much as possible 
     */
    void buildTree_();

    /**
     * Report a duration in the most intuitive units (largest being milliseconds)
     * @return          The string of the unit for this duration 
     */
    const char* normalizeDuration_(time_t& dur_ns) const;

    /**
     * Variable to store the total time taken in each process
     */
    std::map<const char*, TimerTotal> totals_;

    /**
     * Variable for the total number of independent processes measured
     */
    int base_count_ = 1;

    /**
     * Parse a function name so that it reads well to a human, and fits inside the summary portion 
     */
    std::string parseFunctionName(std::string name, bool check_namespace = true);
};

}   // namespace cpp_timer

#endif
