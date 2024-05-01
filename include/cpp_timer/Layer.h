#ifndef TIMER_LAYER_HH_
#define TIMER_LAYER_HH_

#include <string>
#include "cpp_timer/typedefs.h"

namespace cpp_timer{

struct Layer{
    const char* name;
    int layer_index;
    int call_count = 1;
    int call_idx = 0;
    int child_idx = 0;
    ParentPtr parent;
    layerMap children;
    chronoDuration duration = chronoDuration(0);

    // Layer Comparison Functions
    static bool compareLayerByName(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->name < L2->name;
    }

    static bool compareLayerByAverage(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return (float)L1->duration.count()/(float)L1->call_count > (float)L2->duration.count()/(float)L2->call_count;
    }

    static bool compareLayerByTotal(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->duration.count() > L2->duration.count();
    }

    static bool compareLayerByCallCount(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->call_count > L2->call_count;
    }

    static bool compareLayerByCallOrder(const cpp_timer::LayerPtr& L1, const cpp_timer::LayerPtr& L2){
        return L1->call_idx < L2->call_idx;
    }
};

} // end namespace cpp_timer

#endif