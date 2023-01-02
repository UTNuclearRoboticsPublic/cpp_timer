///////////////////////////////////////////////////////////////////////////////
//      Title     : Timer.cpp
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

#include <map>
#include <iomanip>
#include <algorithm>
#include "cpp_timer/Timer.h"

using std::string;
using std::cout;
using std::endl;

namespace{ 
    std::map<string, string> colours = {{"red",     "\e[1;31m"}, 
                                        {"green",   "\e[1;32m"},
                                        {"yellow",  "\e[1;33m"},
                                        {"blue",    "\e[1;34m"},
                                        {"magenta", "\e[1;35m"},
                                        {"cyan",    "\e[1;36m"},
                                        {"white",   "\e[1;37m"},
    };

    string reset = "\e[0m";

    string parseFunctionName(string name){
        // Generally a function name has the form 
        // type namespace1::namespace2::...::namespacen::functionName (args)
        //     ^                                        ^             ^
        // The caret symbols above show where the characters of interest are

        // Find the first instance of a space
        size_t space  = name.find(" ");
        if (space == string::npos){
            return name.substr(0, 31);
        }

        // Find the fist instance of a "("
        size_t bracket = name.find("(", space+1);
        if (bracket == string::npos){
            return name;
        }
        // Find the last '::' before this "("
        size_t last_pos = name.find_last_of(":", bracket);
        if (last_pos == string::npos){
            last_pos = space;
        }

        // Return just the simplified version of the function name
        string name_simple = name.substr(last_pos + 1, bracket - last_pos - 1);

        // Keep the name inside 32 characters for formatting
        return name_simple.substr(0,31);
    }

    typedef std::pair<string, std::pair<int, cpp_timer::chronoDuration>> totalVal;

    // Total Comparison Functions
    bool compareTotalByName(totalVal P1, totalVal P2){
        return P1.first < P2.first;
    }

    bool compareTotalByAverage(totalVal P1, totalVal P2){
        return P1.second.second.count()/(float)P1.second.first > P2.second.second.count()/(float)P2.second.first;
    }

    bool compareTotalByTotal(totalVal P1, totalVal P2){
        return P1.second.second.count() > P2.second.second.count();
    }

    bool compareTotalByCallCount(totalVal P1, totalVal P2){
        return P1.second.first > P2.second.first;
    }

    // Layer Comparison Functions
    bool compareLayerByName(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->name < L2->name;
    }

    bool compareLayerByAverage(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->duration.count()/(float)L1->call_count > L2->duration.count()/(float)L2->call_count;
    }

    bool compareLayerByTotal(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->duration.count() > L2->duration.count();
    }

    bool compareLayerByCallCount(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->call_count > L2->call_count;
    }

    bool compareLayerByCallOrder(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->call_idx < L2->call_idx;
    }
}

namespace cpp_timer{

// ================================================================================
// ================================================================================

Timer::Timer() {
    start_times_.reserve(10);
    current_layer_ = LayerPtr(new Layer);
    current_layer_->layer_index = 0;
    current_layer_->name = "__TIMER_BASE_LAYER__";

    all_layers_.reserve(100);
    all_layers_.push_back(current_layer_);

    tictocs_.reserve(500);

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
    std::unique_lock<std::mutex> tree_lock(tree_mtx_);
    tictocs_.emplace_back(function_name, std::chrono::steady_clock::now(), TicLabel::TICK);
}

// ================================================================================
// ================================================================================

void Timer::toc(const char* function_name){ 
    {
        std::unique_lock<std::mutex> tree_lock(tree_mtx_); 
        tictocs_.emplace_back(function_name, std::chrono::steady_clock::now(), TicLabel::TOCK);  
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
        current_layer_ = current_layer_->parent;
    }

    LayerPtr* current_layer_ptr_ptr = &current_layer_;

    // Show the full function tree
    std::cout << colours["green"] << "\n============================= FUNCTION BREAKDOWN ==============================\n" << reset;
    printLayer_(current_layer_, breakdown_order);

    // Show the total time spent on each function, regardless of parent/child relations
    std::cout << colours["green"]   << "\n\n==================================== SUMMARY ===================================\n";
    std::cout << colours["magenta"] << "\t\t\t\tTotal Time   |   Times Called   |   Average Time\n" << reset;
    
    // Sort the totals by the sorting mechanism presented
    timerTotal totals = getTotals_(current_layer_);
    std::vector<totalVal> sorted_times;
    for (const auto& entry : totals){
        sorted_times.push_back(entry);
    }

    bool (*comp)(totalVal, totalVal);
    switch(total_order){
        default:
        case BY_NAME:
            comp = &compareTotalByName;
            break;

        case BY_TOTAL:
            comp = &compareTotalByTotal;
            break;

        case BY_AVERAGE:
            comp = &compareTotalByAverage;
            break;

        case BY_CALL_COUNT:
            comp = &compareTotalByCallCount;
            break;
    }
    std::sort(sorted_times.begin(), sorted_times.end(), *comp);

    for (const std::pair<std::string, std::pair<int, chronoDuration>> &p : sorted_times){
        std::string name    = parseFunctionName(p.first) + ":";
        long int total_time = p.second.second.count();
        int call_count      = p.second.first;
        long int avg_time   = total_time/call_count;

        std::string avg_unit     = normalizeDuration_(avg_time);        
        std::string total_unit   = normalizeDuration_(total_time);
        std::string bar = colours["magenta"] + "   |   " + colours["cyan"];

        // Formatted output
        std::cout << reset << std::setw(32) << std::left << name;
        std::cout << colours["cyan"]  << std::setw(7) << std::right << total_time << total_unit << bar;
        std::cout << std::setw(12) << call_count << bar;
        std::cout << std::setw(9) << avg_time << avg_unit << "\n";
    }
    std::cout << reset << " " << std::endl;

    // Restart the tree thread
    tree_thread_ = std::thread(&Timer::buildTree_, this);
    while(concluded_);
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
        current_layer_->parent->children.at(label.name)->duration += duration;
        current_layer_ = current_layer_->parent;
    }

    // Now that we've used the most recent start time, we can get rid of it
    start_times_.pop_back();
}

// ================================================================================
// ================================================================================

void Timer::buildTree_(){
    concluded_ = false;
    while (not (concluded_ && tictocs_.empty())){
        // Create vectors to hold the tic and toc vectors
        std::vector<TicLabel> tictoc_copy;

        {
            // Ready a vector which we will put back into the system 
            std::vector<TicLabel> new_tictocs_;
            size_t size = tictocs_.size();
            new_tictocs_.reserve(tictocs_.size());

            // Protect the tictocs_ vectors from race conditions
            std::unique_lock<std::mutex> tree_lock(tree_mtx_);
            
            // Build up the tree if we've collected enough data
            tree_contition_.wait(tree_lock, [this](){return concluded_ || (tictocs_.size() > 500);});

            // Do an Indiana Jones artifact swap
            tictoc_copy = std::exchange(tictocs_, new_tictocs_);
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

std::string Timer::normalizeDuration_(long int& duration) const{
    static const std::array<std::string,3> units = {" ns", " us", " ms"};
    size_t div_count = 0;
    while (duration > 1000 && div_count < (units.size() - 1)){
        duration /= 1000;
        ++div_count;
    }

    return units.at(div_count);
}

// ================================================================================
// ================================================================================

void Timer::printLayer_(const LayerPtr& layer, SummaryOrder order, long int prev_duration){
    if (layer->children.empty()) return;

    // Sort the individual layers by the specified order
    std::vector<LayerPtr> sorted_layers;
    sorted_layers.reserve(layer->children.size());
    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        sorted_layers.push_back(p.second);
    }

    bool(*comp)(const LayerPtr&, const LayerPtr&);
    switch (order){
        case BY_NAME:
            comp = &compareLayerByName;
            break;

        case BY_TOTAL:
            comp = &compareLayerByTotal;
            break;

        case BY_AVERAGE:
            comp = &compareLayerByAverage;
            break;

        case BY_CALL_COUNT:
            comp = &compareLayerByCallCount;
            break;

        default:
            comp = &compareLayerByCallOrder;
    }
    std::sort(sorted_layers.begin(), sorted_layers.end(), comp);

    for (const LayerPtr& L : sorted_layers){
        // Get the child layer info
        std::string name  = parseFunctionName(L->name);
        LayerPtr child    = L;
        long int duration = std::chrono::duration_cast<std::chrono::nanoseconds>(L->duration).count();
        long int avg_dur  = duration/L->call_count;
        long int ns_dur   = duration;
        prev_duration    -= duration;

        std::string unit = normalizeDuration_(duration);
        std::string avg_unit = normalizeDuration_(avg_dur); 

        std::ostringstream left_side, right_side;

        // To make it look pretty
        if (layer->layer_index)
            left_side << colours["magenta"] << std::setw(5*(layer->layer_index + 1)) << std::right << "|--- " << reset;
        else {
            std::cout << '\n';
            left_side << colours["green"] << base_count_++ << ": " << reset;
        }
        left_side << name << colours["blue"] << " (" << L->call_count << "): " << colours["cyan"] << duration << unit;
        std::cout << std::setw(93) << std::left << left_side.str() << " " << colours["magenta"];  

        // Right side contains total runtime of each layer
        right_side << "(" << avg_dur << avg_unit << ")";
        std::cout << std::setw(10) << std::right << right_side.str() << reset << '\n';

        // Recursively print the next layer
        printLayer_(child, order, ns_dur);

        // After all recursive calls have finished, print a dividing line
        if (layer->layer_index == 0)
            cout << colours["white"] << "_______________________________________________________________________________\n";
    }

    // If the function body was at least 1ns, report it
    if (prev_duration > 0){
        long int prev_avg_dur = prev_duration/layer->call_count;
        std::string unit = normalizeDuration_(prev_duration);
        std::string function_unit = normalizeDuration_(prev_avg_dur);

        std::ostringstream left_side, right_side;
        left_side << colours["magenta"] << std::setw(5*(layer->layer_index + 1)) << "|--- " << reset;
        left_side << "Function Body: " << colours["cyan"] << prev_duration << function_unit;
        right_side << "(" << prev_avg_dur << unit << ")";

        std::cout << std::setw(87) << std::left << left_side.str();
        std::cout << colours["magenta"] << std::setw(10) << std::right << right_side.str() << reset << '\n';
    }
}

// ================================================================================
// ================================================================================

timerTotal Timer::getTotals_(LayerPtr layer){

    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        // Get the child layer info
        std::string name = p.first;
        LayerPtr child   = p.second;

        // Determine if the function is recursive
        LayerPtr parent = layer;
        bool is_recursive = false;
        while (parent != all_layers_[0] and not is_recursive){
            is_recursive = (parent->name == name);
            parent = parent->parent;
        }

        // Note that subsequent recursive calls do not increase total time
        if (totals_.count(name)){
            totals_[name].first  += layer->children[name]->call_count;
            totals_[name].second += layer->children[name]->duration * (not is_recursive);

        // If we do not have this label yet, create a new entry for it
        }else{
            totals_[name] = std::pair<int, chronoDuration>();
            totals_[name].first  = layer->children[name]->call_count;
            totals_[name].second = layer->children[name]->duration;
        }

        // If this child has children, repeat
        if (not child->children.empty()){
            getTotals_(child);
        }
    }

    return totals_;
}

} // namespace cpp_timer
