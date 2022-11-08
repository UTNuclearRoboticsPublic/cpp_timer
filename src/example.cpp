#include <iostream>
#include "cpp_timer/Timer.h"

static cpp_timer::Timer timer;

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

void parentFunction(){
    // Use the macro to automatically get the parent function name
    TIMER_TIC;

    // You can place timers around function calls if you want
    for (int i = 0; i < 10000; i++){
        timer.tic("childFunction1");
        childFunction1();
        timer.toc("childFunction1");
    }

    for (int i = 0; i < 100; i++){
        timer.tic("childFunction2");
        childFunction2();
        timer.toc("childFunction2");
    }

    // Macro toc
    TIMER_TOC;
}

// Recursive functions depend on where you put the toc call
int fibonacci_deep(int n){
    TIMER_TIC;

    if (n > 1){
        int return_val = fibonacci_deep(n-1) + fibonacci_deep(n-2);
        TIMER_TOC;
        return return_val;
    }else{
        TIMER_TOC;
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

int main(){
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
    timer.tic("fibonacci_outside");
    fibonacci_flat(5);
    fibonacci_deep(5);
    timer.toc("fibonacci_outside");

    // Here's a more advanced useage used to measure function call overhead
    for (int i = 0; i < 100000; i++){
        timer.tic("function_call_overhead");
        dummy(1, {2, 3});
        timer.toc("function_return_overhead");
    }

    // And here we can see the general overhead of calling tic and toc
    auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < 1e6; i++){
        timer.tic("tictoc overhead");
        timer.tic("dummy");
        timer.toc("dummy");
        timer.toc("tictoc overhead");
    }
    auto t2 = std::chrono::steady_clock::now();
    std::cout << "Tictoc overhead should be " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl; 

    timer.summary(cpp_timer::Timer::SummaryOrder::BY_TOTAL, cpp_timer::Timer::SummaryOrder::BY_CALL_ORDER);
    return 0;
}
