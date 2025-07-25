#include <iostream>
#include "cpp_timer/Timer.h"

static cpp_timer::Timer timer;

using namespace std::chrono_literals;

// Defining a TIMER_INSTANCE allows you to use Timer's Macros: TIMER_TIC and TIMER_TOC
#define TIMER_INSTANCE timer

// Example of using the timer macros to automatically get the function name
void expensiveFunction(std::vector<double> &vec){
    TIMER_TIC;
    while(!vec.empty()){
        vec.erase(vec.begin());
    }
    TIMER_TOC;
}

// Example of internally timing an efficient function
void efficientFuction(std::vector<double> &vec){
    timer.tic("efficientFunction");
    vec.clear();
    timer.toc("efficientFunction");
}

void sleeperFunction(){
    std::this_thread::sleep_for(1s);
}

// Example of using the timer to time blocks of code within a scope
void childFunction1(){
    timer.tic("loop_1");
    for (int i = 0; i < 10000; i++){
        int j = i/2.0;
    }
    timer.toc("loop_1");

    timer.tic("loop_2");
    for (int k = 0; k < 500; k++){
        std::string some_string = std::to_string(k);
    }
    timer.toc("loop_2");
}

// Example of using a timer to time function call overhead
void dummy(int a, std::vector<float> b, double c = M_PI){
    timer.toc("function_call_overhead");
    timer.tic("function_return_overhead");
}

void childFunction2(){
    std::vector<double> dummy_vec2(10000);
    std::vector<double> dummy_vec3(10000);
    expensiveFunction(dummy_vec2);
    efficientFuction(dummy_vec3);

    // You can time the same function in multiple places in code
    timer.tic("childFunction1");
    childFunction1();
    timer.toc("childFunction1");
}

void multithreadedFunction(){
    std::vector<std::thread> threads(5);
    volatile uint64_t sum = 0;
    auto work = [&sum](){
        for (int i = 0; i < 1e8; i++){
            sum++;
        }
    };

    for (int i = 0; i < 5; i++){
        threads[i] = std::thread(work);
    }

    for (auto& thread : threads){
        thread.join();
    }
}

void granularMultithreadedFunction(){
    auto _ = timer.scopedTic(BOOST_CURRENT_FUNCTION);
    std::vector<std::thread> threads(5);
    volatile uint64_t sum = 0;
    auto work = [&sum](int idx){
        timer.tic("work");
        for (int i = 0; i < 1e8; i++){
            sum++;
        }
        if (idx == 3) {
            timer.tic("thread 3 work");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timer.toc("thread 3 work");
        }
        timer.toc("work");
    };

    timer.enterMultithreadedRegion();
    for (int i = 0; i < 5; i++){
        threads[i] = std::thread(work, i);
    }

    for (auto& thread : threads){
        thread.join();
    }
    timer.exitMultithreadedRegion();
}

void parentFunction(){
    // Use the macro to automatically get the parent function name
    TIMER_TIC;

    // You can place timers around function calls if you want
    for (int i = 0; i < 10000; i++){
        timer.tic("childFunction1");
        childFunction1();
        timer.toc("childFunction1");
    }

    // This only counts as a single function call
    timer.tic("childFunction2");
    for (int i = 0; i < 100; i++){
        childFunction2();
    }
    timer.toc("childFunction2");

    // Macro toc
    TIMER_TOC;
}

// Recursive functions depend on where you put the toc call
int fibonacci_deep(int n){
    STIC;

    if (n > 1){
        return fibonacci_deep(n-1) + fibonacci_deep(n-2);;
    }else{
        return 1;
    }
}

int fibonacci_flat(int n){
    TIMER_TIC;

    if (n > 1){
        TIMER_TOC;
        return fibonacci_flat(n-1) - fibonacci_flat(n-2);
    }else{
        TIMER_TOC;
        return 1;
    }
}

void benchmarkTicAndToc(){
    // Let's benchmark against a basic std::chrono time point check
    const std::chrono::nanoseconds::rep loop_count = 1e6;
    std::chrono::nanoseconds dur(0);
    auto sum = 0;
    for (int i = 0; i < loop_count; i++){
        const auto t1 = std::chrono::steady_clock::now();
        const auto unused_t = std::chrono::steady_clock::now();
        const auto t2 = std::chrono::steady_clock::now();
        dur += (t2 - t1);
        sum += unused_t.time_since_epoch().count();
    }
    dur /= loop_count;
    std::cout << "Internal benchmark: \n";
    std::cout << "\t * Total time it took for chrono: " << dur.count() << " ns\n";

    std::chrono::nanoseconds tic_dur(0);
    std::chrono::nanoseconds toc_dur(0);
    for (int i = 0; i < loop_count; i++){
        // Use chrono to measure tic and toc times
        const auto tic_t1 = std::chrono::steady_clock::now();
        timer.tic("Test TicToc Duration");
        const auto tic_t2 = std::chrono::steady_clock::now();
        const auto toc_t1 = std::chrono::steady_clock::now();
        timer.toc("Test TicToc Duration");
        const auto toc_t2 = std::chrono::steady_clock::now();

        // Subtract the part that we calculated comes from std::chrono
        tic_dur += (tic_t2 - tic_t1 - dur);
        toc_dur += (toc_t2 - toc_t1 - dur);
    }
    std::cout << "\t * On average a tic took " << tic_dur.count()/loop_count << " ns\n";
    std::cout << "\t * On average a toc took " << toc_dur.count()/loop_count << " ns\n";

    std::chrono::nanoseconds cpu_tic_dur(0);
    std::chrono::nanoseconds cpu_toc_dur(0);
    for (int i = 0; i < loop_count; i++){
        // Use chrono to measure tic and toc times
        const auto tic_t1 = std::chrono::steady_clock::now();
        timer.ticCpu("Test TicToc Duration");
        const auto tic_t2 = std::chrono::steady_clock::now();
        const auto toc_t1 = std::chrono::steady_clock::now();
        timer.tocCpu("Test TicToc Duration");
        const auto toc_t2 = std::chrono::steady_clock::now();

        // Subtract the part that we calculated comes from std::chrono
        cpu_tic_dur += (tic_t2 - tic_t1 - dur);
        cpu_toc_dur += (toc_t2 - toc_t1 - dur);
    }
    std::cout << "\t * On average a CPU tic took " << cpu_tic_dur.count()/loop_count << " ns\n";
    std::cout << "\t * On average a CPU toc took " << cpu_toc_dur.count()/loop_count << " ns\n";
}

int main(){
    // Compare cpp_timer to bare std::chrono implementation
    benchmarkTicAndToc();

    for (int i = 0; i < 2500; i++){
        std::vector<double> dummy_vec1(10000);
        efficientFuction(dummy_vec1);
    }

    // This function contains many internal timer calls
    parentFunction();

    // Very long function names will be truncated in the final summary
    timer.tic("reallyLongFunctionNameThatDoesntQuiteFit");
    timer.toc("reallyLongFunctionNameThatDoesntQuiteFit");

    // Calling timer outside recursive functions only counts as one call
    timer.tic("fibonacci_flat_outside");
    fibonacci_flat(5);
    timer.toc("fibonacci_flat_outside");
    timer.tic("fibonacci_deep_outside");
    fibonacci_deep(5);
    timer.toc("fibonacci_deep_outside");

    // Here's a more advanced useage used to measure function call overhead
    for (int i = 0; i < 100000; i++){
        timer.tic("function_call_overhead");
        dummy(1, {2, 3});
        timer.toc("function_return_overhead");
    }

    // And here we can see the general overhead of calling tic and toc
    for (int i = 0; i < 1e6; i++){
        timer.tic("tictoc overhead");
        timer.tic("manual_overhead");
        timer.toc("manual_overhead");
        timer.toc("tictoc overhead");

        auto _ = timer.scopedTic("scope_overhead");
    }

    // Here's the difference between real and cpu time
    timer.enterMultithreadedRegion();
    timer.tic("RealVsCpu");

    timer.tic("realTime");
    sleeperFunction();
    timer.toc("realTime");

    timer.ticCpu("cpuTime");
    sleeperFunction();
    timer.tocCpu("cpuTime");

    timer.tic("multithreadedReal");
    multithreadedFunction();
    timer.toc("multithreadedReal");

    timer.ticCpu("multithreadedCpu");
    multithreadedFunction();
    timer.tocCpu("multithreadedCpu");

    timer.toc("RealVsCpu");
    timer.exitMultithreadedRegion();

    granularMultithreadedFunction();

    // Print the overall summary in decreaseing runtime order, and the function breakdown in the order the functions were called
    timer.summary(cpp_timer::Timer::SummaryOrder::BY_TOTAL, cpp_timer::Timer::SummaryOrder::BY_CALL_ORDER);
    return 0;
}
