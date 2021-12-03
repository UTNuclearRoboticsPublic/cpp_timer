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
                                        {"while",   "\e[1;37m"},
    };

    // string reset = "\e[0m";
    string reset = "\e[0;36m";

    string parseFunctionName(string name){
        // Generally a function name has the form 
        // type namespace1::namespace2::...::namespacen::functionName (args)
        //     ^                                        ^             ^
        // The caret symbols above show where the characters of interest are

        // Find the first instance of a space
        size_t space  = name.find(" ");
        if (space == string::npos){
            return name;
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
        return name_simple;
    }
}

namespace cpp_timer{

// ================================================================================
// ================================================================================

Timer::Timer(){
    start_times_.reserve(10);
    current_layer_ = LayerPtr(new Layer);
    current_layer_->layer_index = 0;

    all_layers_.reserve(100);
    all_layers_.push_back(current_layer_);
}

// ================================================================================
// ================================================================================

void Timer::tic(string function_name){
    chronoTime tic_start = std::chrono::steady_clock::now();

    // If there is already a layer for the function in this framework, move to it
    if (current_layer_->children.count(function_name)){
        current_layer_->children[function_name]->call_count++;
        current_layer_ = current_layer_->children[function_name];
    }
    
    // Otherwise, create a new layer with the current layer as the parent
    else{
        LayerPtr new_layer(new Layer);
        all_layers_.push_back(new_layer);

        // Assign the new layer parameters
        new_layer->layer_index = current_layer_->layer_index + 1;
        
        // Update the parent-child relations
        new_layer->parent = current_layer_;
        current_layer_->children[function_name] = new_layer;

        // Set the current layer to be this new layer
        current_layer_ = new_layer;
    }

    // Record the duration of the tic and the start time of the function
    chronoTime tic_end = std::chrono::steady_clock::now();
    current_layer_->parent->child_tic_duration += (tic_end - tic_start);
    start_times_.push_back(tic_end);
}

// ================================================================================
// ================================================================================

void Timer::toc(string function_name){        
    // Find the duration since the last tic
    chronoTime toc_start_time = std::chrono::steady_clock::now();
    chronoDuration duration = toc_start_time - start_times_.back();

    // Move up a layer if possible
    if (current_layer_->layer_index != 0){
        current_layer_->parent->children[function_name]->duration += duration;
        current_layer_ = current_layer_->parent;
    }

    // Now that we've used the most recent start time, we can get rid of it
    start_times_.pop_back();

    // Record the time spent in the toc function
    current_layer_->child_tic_duration += (std::chrono::steady_clock::now() - toc_start_time);
}

// ================================================================================
// ================================================================================

void Timer::summary(){
    // Ensure that all loose ends have been tied up
    assert(current_layer_->layer_index == 0);
    assert(start_times_.empty());

    LayerPtr* current_layer_ptr_ptr = &current_layer_;

    // For each layer, subtract the tic-toc times of child layers
    for (LayerPtr &p : all_layers_){
        p->duration -= getChildTicTocTime_(p);
    }

    // Show the full function tree
    std::cout << colours["cyan"] << "\n============================= FUNCTION BREAKDOWN =============================" << reset;
    printLayer_(current_layer_);

    // Show the total time spent on each function, regardless of parent/child times
    std::cout << colours["cyan"] << "\n=================================== SUMMARY ===================================\n";
    std::cout << "\t\t\t\tTotal Time   |  Times Called   |   Average Time\n" << reset;
    timerTotal totals = getTotals_(current_layer_);
    for (const std::pair<std::string, std::pair<int, chronoDuration>> &p : totals){
        std::string name    = parseFunctionName(p.first);
        long long  duration = p.second.second.count();
        int call_count      = p.second.first;
        long long avg_time  = duration/call_count;

        // For each measurement we need a unit, a value and a colour to print with
        string total_colour, call_colour, avg_colour;
        string total_unit, avg_unit;
        uint total_time;

        if (duration < 1e3){
            total_unit   = " ns";
            total_colour = colours["cyan"]; //colours["green"];
            total_time   = duration;
        }else if (duration < 1e6){
            total_unit   = " us";
            total_colour = colours["cyan"]; //colours["blue"];
            total_time   = duration/1e3;
        }else if (duration < 1e9){
            total_unit   = " ms";
            total_colour = colours["cyan"]; //colours["yellow"];
            total_time   = duration/1e6;
        }else{
            total_unit   = " ms";
            total_colour = colours["cyan"]; //colours["red"];
            total_time   = duration/1e6;
        }

        if (avg_time < 1e3){
            avg_unit   = " ns";
            avg_colour = colours["cyan"]; //colours["green"];
        }else if (avg_time < 1e6){
            avg_unit   = " us";
            avg_colour = colours["cyan"]; //colours["blue"];
            avg_time   = avg_time/1e3;
        }else if (avg_time < 1e9){
            avg_unit   = " ms";
            avg_colour = colours["cyan"]; //colours["yellow"];
            avg_time   = avg_time/1e6;
        }else{
            avg_unit   = " ms";
            avg_colour = colours["cyan"]; //colours["red"];
            avg_time   = avg_time/1e6;
        }

        // Used to align times
        int pre_space_count  = 32 - (name.length() + 1);
        int space_count      =  6 - (int)floor(log10(total_time));
        int call_space_count =  6 - (int)floor(log10(call_count));
        int avg_space_count  =  8 - (int)floor(log10(avg_time));
        
        // Correct for log domain error
        if (total_time == 0) space_count = 6;
        if (avg_time == 0) avg_space_count = 8;

        std::cout << name << ":";
        for(int i = 0; i < pre_space_count; i++) std::cout << " ";
        for(int i = 0; i < space_count; i++) std::cout << " ";
        std::cout << total_colour << total_time << total_unit << reset << "   |";
        for(int i = 0; i < call_space_count + 7; i++) std::cout << " ";
        std::cout << colours["cyan"] << call_count << reset << "   |";
        for(int i = 0; i < avg_space_count + 3; i++) std::cout << " ";
        std::cout << avg_colour << avg_time << avg_unit << reset << std::endl; 
    }
    std::cout << reset << " " << std::endl;
}

// ================================================================================
// ================================================================================

void Timer::printLayer_(const LayerPtr& layer, int prev_duration){
    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        // Get the child layer info
        std::string name  = parseFunctionName(p.first);
        LayerPtr child    = p.second;
        long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(p.second->duration).count();
        prev_duration    -= duration;

        // To make it look pretty
        for(int i = 0; i < layer->layer_index; i++) std::cout << "     ";
        if (layer->layer_index > 0) std::cout << "|--- ";
        else std::cout << "\n";

        std::cout << name << " (" << p.second->call_count << "): " << colours["cyan"] << duration << " ms\n" << reset;

        // If this child has children, repeat
        if (not child->children.empty()){
            printLayer_(child, duration);
        }
    }

    if (prev_duration > 0){
        for(int i = 0; i < layer->layer_index; i++) std::cout << "     ";
        if (layer->layer_index > 0) std::cout << "|--- ";
        std::cout << "other: " << colours["cyan"] << prev_duration << " ms\n" << reset;
    }
}

// ================================================================================
// ================================================================================

timerTotal Timer::getTotals_(LayerPtr layer){
    static timerTotal totals;
    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        // Get the child layer info
        std::string name = p.first;
        LayerPtr child   = p.second;

        if (totals.count(name)){
            totals[name].first  += layer->children[name]->call_count;
            totals[name].second += layer->children[name]->duration;
        }else{
            totals[name] = std::pair<int, chronoDuration>();
            totals[name].first  = layer->children[name]->call_count;
            totals[name].second = layer->children[name]->duration;
        }

        // If this child has children, repeat
        if (not child->children.empty()){
            getTotals_(child);
        }
    }

    return totals;
}

// ================================================================================
// ================================================================================

chronoDuration Timer::getChildTicTocTime_(LayerPtr layer){
    // Get the time spent in tic-tocs for direct descendants
    chronoDuration child_tic_toc_dur = layer->child_tic_duration;

    // Get the time spent in tic-tocs for subsequenct descendents
    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        LayerPtr child = p.second;
        child_tic_toc_dur += getChildTicTocTime_(child);
    }

    return child_tic_toc_dur;
}

} // namespace cpp_timer
