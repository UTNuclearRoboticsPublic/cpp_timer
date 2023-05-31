///////////////////////////////////////////////////////////////////////////////
//      Title     : Timer.cpp
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

#include <map>
#include <iomanip>
#include <iostream>
#include <assert.h>
#include <algorithm>
#include "cpp_timer/Timer.h"
#include "cpp_timer/Ticker.h"
#include "cpp_timer/TimerTotal.h"

using namespace std::string_literals;

using std::cout;
using std::endl;

#define TICTOC_BUFFER_SIZE 1000

constexpr struct {
    const char* red     = "\033[1;31m";
    const char* green   = "\033[1;32m";
    const char* yellow  = "\033[1;33m";
    const char* blue    = "\033[1;34m";
    const char* magenta = "\033[1;35m";
    const char* cyan    = "\033[1;36m";
    const char* white   = "\033[1;37m";
    const char* reset   = "\033[0m";
} colours;

namespace cpp_timer{

// ================================================================================
// ================================================================================

Timer::Timer() {
    current_layer_ = std::make_shared<Layer>();
    current_layer_->layer_index = 0;
    current_layer_->name = "__TIMER_BASE_LAYER__";
    all_layers_.push_back(current_layer_);

    tictocs_.reserve(2*TICTOC_BUFFER_SIZE);

    // Create the tree building thread and wait for it to initialize
    tree_thread_ = std::thread(&Timer::buildTree_, this);
    while(concluded_);
}

// ================================================================================
// ================================================================================

Timer::~Timer(){
    concluded_ = true;
    tree_contition_.notify_one();
    tree_thread_.join();
}

// ================================================================================
// ================================================================================

void Timer::tic(const char* function_name){
    std::lock_guard<std::mutex> tree_lock(tree_mtx_);
    tictocs_.emplace_back(function_name, std::chrono::steady_clock::now(), TicLabel::TICK);
}

// ================================================================================
// ================================================================================

void Timer::toc(const char* function_name){ 
    const auto now = std::chrono::steady_clock::now(); 
    {
        std::lock_guard<std::mutex> tree_lock(tree_mtx_); 
        tictocs_.emplace_back(function_name, now, TicLabel::TOCK);  
    }    
    tree_contition_.notify_one();
}

// ================================================================================
// ================================================================================

void Timer::ticCpu(const char* function_name){
    std::lock_guard<std::mutex> tree_lock(tree_mtx_);
    tictocs_.emplace_back(function_name, getCPUTimePoint_(), TicLabel::TICK);
}

// ================================================================================
// ================================================================================

void Timer::tocCpu(const char* function_name){ 
    const auto now = getCPUTimePoint_();
    {
        std::lock_guard<std::mutex> tree_lock(tree_mtx_); 
        tictocs_.emplace_back(function_name, now, TicLabel::TOCK);  
    }    
    tree_contition_.notify_one();
}

// ================================================================================
// ================================================================================

Ticker Timer::scopedTic(const char* function_name){
    return Ticker(function_name, this);
}

// ================================================================================
// ================================================================================

void Timer::summary(Timer::SummaryOrder total_order, Timer::SummaryOrder breakdown_order){
    // Notify the worker thread to finish with its job
    {
        std::lock_guard<std::mutex> tree_lock(tree_mtx_);
        concluded_ = true;
    }
    tree_contition_.notify_one();
    tree_thread_.join();
    assert(tictocs_.empty());

    // Reset our state to the base layer
    while(current_layer_->layer_index != 0){
        current_layer_ = current_layer_->parent.lock();
    }

    // Show the full function tree
    std::cout << colours.green << "\n============================= FUNCTION BREAKDOWN =============================\n" << colours.reset;
    printLayer_(current_layer_, breakdown_order);

    // Show the total time spent on each function, regardless of parent/child relations
    std::cout << colours.green   << "\n\n==================================== SUMMARY ===================================\n";
    std::cout << colours.magenta <<     "                                Total Time   |   Times Called   |   Average Time\n" << colours.reset;
    
    // Sort the totals by the sorting mechanism presented
    getTotals_(current_layer_);
    std::vector<TimerTotal> sorted_times;
    sorted_times.reserve(totals_.size());
    for (const auto& [name, total] : totals_){
        sorted_times.push_back(total);
    }

    // Assign the correct function for sorting
    bool (*comp)(TimerTotal, TimerTotal) = std::invoke([total_order](){
        switch (total_order){
            case BY_NAME       : return &TimerTotal::compareTotalByName;
            case BY_TOTAL      : return &TimerTotal::compareTotalByTotal;
            case BY_AVERAGE    : return &TimerTotal::compareTotalByAverage;
            case BY_CALL_COUNT : return &TimerTotal::compareTotalByCallCount;
            default            : throw std::runtime_error("Cannot sort function totals by call order!");
        }
    });

    std::sort(sorted_times.begin(), sorted_times.end(), comp);

    for (const TimerTotal &T : sorted_times){
        std::string_view name = parseFunctionName(T.name);
        time_t total_time   = T.duration.count();
        int call_count      = T.call_count;
        time_t avg_time     = total_time/call_count;

        const std::string_view avg_unit   = normalizeDuration_(avg_time);        
        const std::string_view total_unit = normalizeDuration_(total_time);
        const std::string bar = colours.magenta + "   |   "s + colours.cyan;

        // Formatted output
        std::cout << colours.reset << std::setw(31) << std::left << name << ":";
        std::cout << colours.cyan  << std::setw(7) << std::right << total_time << total_unit << bar;
        std::cout << std::setw(12) << call_count << bar;
        std::cout << std::setw(9)  << avg_time << avg_unit << "\n";
    }
    std::cout << colours.reset << " " << std::endl;

    // Restart the tree thread
    tree_thread_ = std::thread(&Timer::buildTree_, this);
    base_count_  = 1;
    while(concluded_);
}

// ================================================================================
// ================================================================================

chronoTime Timer::getCPUTimePoint_() const{
    static constexpr auto multiplier = chronoTime::duration::period::den / chronoTime::duration::period::num;

    const std::clock_t time_point = std::clock();
    const chronoDuration chrono_ticks(time_point * multiplier / CLOCKS_PER_SEC);
    const chronoTime now(chrono_ticks);

    return now;
}

// ================================================================================
// ================================================================================

void Timer::treeTic_(TicLabel label){
    // If there is already a layer for the function in this framework, move to it
    if (current_layer_->children.count(label.name)){
        current_layer_->children[label.name]->call_count++;
        current_layer_ = current_layer_->children[label.name];
    }
    
    // Otherwise, create a new layer with the current layer as the parent
    else{
        LayerPtr new_layer(new Layer);
        all_layers_.push_back(new_layer);

        // Assign the new layer parameters
        new_layer->layer_index = current_layer_->layer_index + 1;
        new_layer->name = label.name;
        new_layer->call_idx = current_layer_->child_idx++;
        
        // Update the parent-child relations
        new_layer->parent = current_layer_;
        current_layer_->children[label.name] = new_layer;

        // Set the current layer to be this new layer
        current_layer_ = new_layer;
    }

    // Record the duration of the tic and the start time of the function
    start_times_.push_back(label.stamp);
}

// ================================================================================
// ================================================================================

void Timer::treeToc_(TicLabel label){
    // Find the duration since the last tic
    chronoDuration duration = label.stamp - start_times_.back();

    // Move up a layer if possible
    if (current_layer_->layer_index != 0){
        try{
            current_layer_->parent.lock()->children.at(label.name)->duration += duration;
            current_layer_ = current_layer_->parent.lock();
        }catch(std::out_of_range& e){
            std::cout << "Cannot toc the label \"" << label.name << "\" when the current label is \"" << current_layer_->name << "\"\n";
            std::cout << "Current layer parent is " << current_layer_->parent.lock()->name << " with children\n";
            for (const auto& [name, _] : current_layer_->parent.lock()->children){
                std::cout << "\t" << name << "\n";
            }
            throw;
        }
    }

    // Now that we've used the most recent start time, we can get rid of it
    start_times_.pop_back();
}

// ================================================================================
// ================================================================================

void Timer::buildTree_(){
    concluded_ = false;
    while (not (concluded_ && tictocs_.empty())){
        // Create a large vector to hold all the current tictocs (discourages copying at runtime)
        std::vector<TicLabel> tictoc_copy;
        tictoc_copy.reserve(2*TICTOC_BUFFER_SIZE);

        {
            // Protect the tictocs_ vectors from race conditions
            std::unique_lock<std::mutex> tree_lock(tree_mtx_);
            
            // Build up the tree if we've collected enough data
            tree_contition_.wait(tree_lock, [this](){return concluded_ || (tictocs_.size() > TICTOC_BUFFER_SIZE);});

            // Do an Indiana Jones artifact swap
            std::swap(tictoc_copy, tictocs_);
        }

        // Add all the current timestamps to the tree
        for (const TicLabel& label : tictoc_copy){
            switch (label.type){
                case TicLabel::TICK:
                    treeTic_(label);
                    break;

                case TicLabel::TOCK:
                    treeToc_(label);
                    break;
            }
        }
    };
}

// ================================================================================
// ================================================================================

std::string_view Timer::normalizeDuration_(time_t& duration) const{
    static constexpr std::array units = {" ns", " us", " ms"};
    size_t div_count = 0;
    while (duration > 1000 && div_count < (units.size() - 1)){
        duration /= 1000;
        ++div_count;
    }

    return units.at(div_count);
}

// ================================================================================
// ================================================================================

void Timer::printLayer_(const LayerPtr& layer, SummaryOrder order, time_t prev_duration){
    if (layer->children.empty()) return;

    // Sort the individual layers by the specified order
    std::vector<LayerPtr> sorted_layers;
    sorted_layers.reserve(layer->children.size());
    for (const auto& [name, child] : layer->children){
        sorted_layers.push_back(child);
    }

    // Get the appropriate sorting function
    bool(*comp)(const LayerPtr&, const LayerPtr&) = std::invoke([order](){
        switch (order){
            default:
            case BY_NAME       : return &Layer::compareLayerByName;
            case BY_TOTAL      : return &Layer::compareLayerByTotal;
            case BY_AVERAGE    : return &Layer::compareLayerByAverage;
            case BY_CALL_COUNT : return &Layer::compareLayerByCallCount;
            case BY_CALL_ORDER : return &Layer::compareLayerByCallOrder;
        }
    });

    std::sort(sorted_layers.begin(), sorted_layers.end(), comp);

    for (const LayerPtr& child : sorted_layers){
        // Get the child layer info
        std::string_view name  = parseFunctionName(child->name);
        time_t duration      = std::chrono::duration_cast<std::chrono::nanoseconds>(child->duration).count();
        time_t avg_dur       = duration/child->call_count;
        prev_duration         -= duration;

        auto normalized_duration = duration;
        const std::string_view unit = normalizeDuration_(normalized_duration);
        const std::string_view avg_unit = normalizeDuration_(avg_dur); 

        std::ostringstream left_side, right_side;

        // To make it look pretty
        if (layer->layer_index)
            left_side << colours.magenta << std::setw(5*(layer->layer_index + 1)) << std::right << "|--- " << colours.reset;
        else {
            std::cout << '\n';
            left_side << colours.green << base_count_++ << ": " << colours.reset;
        }
        left_side << name << colours.blue << " (" << child->call_count << "): " << colours.cyan << normalized_duration << unit;
        std::cout << std::setw(92) << std::left << left_side.str() << " " << colours.magenta;  

        // Right side contains average runtime of each layer
        right_side << "(" << avg_dur << avg_unit << ")";
        std::cout << std::setw(10) << std::right << right_side.str() << colours.reset << '\n';

        // Recursively print the next layer
        printLayer_(child, order, duration);

        // After all recursive calls have finished, print a dividing line
        if (layer->layer_index == 0)
            cout << colours.white << "______________________________________________________________________________\n";
    }

    // If the function body was at least 1ns, report it
    if (prev_duration > 0){
        time_t prev_avg_dur = prev_duration/layer->call_count;
        const std::string_view unit = normalizeDuration_(prev_duration);
        const std::string_view function_unit = normalizeDuration_(prev_avg_dur);

        std::ostringstream left_side, right_side;
        left_side << colours.magenta << std::setw(5*(layer->layer_index + 1)) << "|--- " << colours.reset;
        left_side << "Function Body: " << colours.cyan << prev_duration << unit;
        right_side << "(" << prev_avg_dur << function_unit << ")";

        std::cout << std::setw(86) << std::left << left_side.str();
        std::cout << colours.magenta << std::setw(10) << std::right << right_side.str() << colours.reset << '\n';
    }
}

// ================================================================================
// ================================================================================

// timerTotal Timer::getTotals_(LayerPtr layer){
void Timer::getTotals_(LayerPtr layer){

    std::vector<TimerTotal> output;

    for (const auto& [name, child] : layer->children){
        
        // Determine if the function is recursive
        LayerPtr parent = layer;
        bool is_recursive = false;
        while (parent != all_layers_[0] and not is_recursive){
            is_recursive = (parent->name == name);
            parent = parent->parent.lock();
        }

        // Note that subsequent recursive calls do not increase total time
        if (totals_.count(name)){
            totals_[name].call_count += layer->children[name]->call_count;
            totals_[name].duration   += layer->children[name]->duration * (not is_recursive);

        // If we do not have this label yet, create a new entry for it
        }else{
            const LayerPtr& new_layer = layer->children[name];
            totals_.insert({name, TimerTotal(name, new_layer->call_count, new_layer->duration)});
        }

        // If this child has children, repeat
        if (not child->children.empty()){
            getTotals_(child);
        }
    }
}

// ================================================================================
// ================================================================================

std::string_view Timer::parseFunctionName(std::string_view name){
    // Generally a function name has the form 
    // type namespace1::namespace2::...::namespacen::functionName (args)
    //     ^                                        ^             ^
    // The caret symbols above show where the characters of interest are

    // Find the first instance of a space
    size_t space  = name.find(" ");

    // If no space was found, return the entire string truncated to fit
    if (space == std::string::npos){
        return name.substr(0, 30);
    }

    // Find the fist instance of a "("
    size_t bracket = name.find("(", space+1);
    if (bracket == std::string::npos){
        return name;
    }

    // Find the last '::' before this "("
    size_t last_pos = name.find_last_of(":", bracket);
    if (last_pos == std::string::npos){
        last_pos = space;
    }

    // Return just the simplified version of the function name
    auto name_simple = name.substr(last_pos + 1, bracket - last_pos - 1);

    // Keep the name inside 31 characters for formatting
    return name_simple.substr(0,30);
}

} // namespace cpp_timer
