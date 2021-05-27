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

#include "cpp_timer/Timer.h"

using std::string;

namespace cpp_timer{

// ================================================================================
// ================================================================================

Timer::Timer(){
    start_times_.reserve(10);
    current_layer_ = LayerPtr(new Layer);
    current_layer_->layer_index = 0;
}

// ================================================================================
// ================================================================================

void Timer::tic(string function_name){
    // If there is already a layer for the function in this framework, move to it
    if (current_layer_->children.count(function_name)){
        current_layer_ = current_layer_->children[function_name];
    }
    
    // Otherwise, create a new layer with the current layer as the parent
    else{
        LayerPtr new_layer(new Layer);

        // Assign the new layer parameters
        new_layer->layer_index = current_layer_->layer_index + 1;
        
        // Update the parent-child relations
        new_layer->parent = current_layer_;
        current_layer_->children[function_name] = new_layer;

        // Set the current layer to be this new layer
        current_layer_ = new_layer;
    }

    // Record the start time
    start_times_.push_back(std::chrono::high_resolution_clock::now());
}

// ================================================================================
// ================================================================================

long int Timer::toc(string function_name){        
    // Find the duration since the last tic
    chronoTime now  = std::chrono::high_resolution_clock::now();
    long int duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_times_.back()).count();

    // Move up a layer if possible
    if (current_layer_->layer_index != 0){
        current_layer_->parent->durations[function_name] += duration;
        current_layer_ = current_layer_->parent;
    }

    // Now that we've used the most recent start time, we can get rid of it
    start_times_.pop_back();
    return duration;
}

// ================================================================================
// ================================================================================

void Timer::summary(){
    // Ensure that all loose ends have been tied up
    assert(current_layer_->layer_index == 0);
    assert(start_times_.empty());

    // Show the full function tree
    std::cout << "\n===== FUNCTION BREAKDOWN =====\n";
    printLayer_(current_layer_);

    // Show the total time spent on each function, regardless of parent/child times
    std::cout << "\n===== SUMMARY =====\n";
    durationMap totals = getTotals_(current_layer_);
    for (std::pair<std::string, double> p : totals){
        std::string name = p.first;
        int duration     = round(p.second/1000);

        // How many tabs to do before printing time
        int tab_count = (name.length() + 1)/8;
        // Used to align times
        int space_count = 6 - (int)floor(log10(p.second/1000.0));

        std::cout << name << ":";
        for(int i = 0; i < 4-tab_count; i++) std::cout << "\t";
        for(int i = 0; i < space_count; i++) std::cout << " ";
        printf("%d ms\n", duration);
    }
    std::cout << std::endl;
}

// ================================================================================
// ================================================================================

void Timer::printLayer_(const LayerPtr& layer, int prev_duration){
    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        // Get the child layer info
        std::string name = p.first;
        LayerPtr child   = p.second;
        int duration     = (int)round(layer->durations[name]/1000.0);
        prev_duration   -= duration;

        // To make it look pretty
        for(int i = 0; i < layer->layer_index; i++) std::cout << "     ";
        if (layer->layer_index > 0) std::cout << "|--- ";
        std::cout << name << ": " << duration << " ms\n";

        // If this child has children, repeat
        if (not child->children.empty()){
            printLayer_(child, duration);
        }
    }

    if (prev_duration > 0){
        for(int i = 0; i < layer->layer_index; i++) std::cout << "     ";
        if (layer->layer_index > 0) std::cout << "|--- ";
        std::cout << "other: " << prev_duration << " ms\n";
    }
}

// ================================================================================
// ================================================================================

durationMap Timer::getTotals_(LayerPtr layer){
    static durationMap totals;
    for (const std::pair<std::string, LayerPtr> &p : layer->children){
        // Get the child layer info
        std::string name = p.first;
        LayerPtr child   = p.second;

        if (totals.count(name)){
            totals[name] += layer->durations[name];
        }else{
            totals[name] = layer->durations[name];
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

} // namespace cpp_timer
