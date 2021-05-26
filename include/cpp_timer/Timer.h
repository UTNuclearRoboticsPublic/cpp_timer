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


#define chronoTime std::chrono::high_resolution_clock::time_point
#define LayerPtr   std::shared_ptr<Layer>

struct Layer{
    int layer_index;
    std::map<std::string, long int> durations;
    std::map<std::string, LayerPtr> children;
    LayerPtr parent;
};

class Timer{
public:
    Timer();

    void tic(std::string function_name);

    double toc(std::string function_name);

    void summary();

private:
    LayerPtr current_layer_;
    std::vector<chronoTime> start_times_;

    // Recursively print a layer and all of it's children
    void printLayer_(const LayerPtr& layer, int prev_duration = 0);

    std::map<std::string, double> getTotals_(LayerPtr layer);
};

#undef LayerPtr
#undef chronoTime

#endif
