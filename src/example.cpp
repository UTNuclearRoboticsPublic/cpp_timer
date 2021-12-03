#include <iostream>
#include "cpp_timer/Timer.h"

static cpp_timer::Timer timer;

// Defining a TIMER_INSTANCE allows you to use Timer's Macros: TIMER_TIC and TIMER_TOC
#define TIMER_INSTANCE timer

void expensiveFunction(std::vector<double> &vec){
    TIMER_TIC;
    while(!vec.empty()){
        vec.erase(vec.begin());
    }
    TIMER_TOC;
}

void efficientFuction(std::vector<double> &vec){
    timer.tic("efficientFunction");
    vec.clear();
    timer.toc("efficientFunction");
}

void childFunction1(){
    timer.tic("childFunction1");
    for (int i = 0; i < 10000; i++){
        int j = i/2.0;
    }
    timer.toc("childFunction1");
}

void dummy(int a, std::vector<float> b, double c = M_PI){
    timer.toc("function_call_overhead");
    timer.tic("function_return_overhead");
}

void childFunction2(){
    std::vector<double> dummy_vec2(10000);
    std::vector<double> dummy_vec3(10000);
    expensiveFunction(dummy_vec2);
    efficientFuction(dummy_vec3);

    // Nesting functions also works
    childFunction1();
}

void parentFunction(){
    // Use the macro to automatically get the parent function name
    TIMER_TIC;

    // You can also label indiviual sections of code for testing
    timer.tic("Loop1");
    for (int i = 0; i < 100000; i++){
        // This function has the timer call inside the function definition
        childFunction1();
    }
    timer.toc("Loop1");

    // You can place timers around function calls if you want
    for (int i = 0; i < 100; i++){
        timer.tic("childFunction2");
        childFunction2();
        timer.toc("childFunction2");
    }

    // You can also use the same label in multiple places to test the same function
    timer.tic("childFunction2");
    childFunction2();
    timer.toc("childFunction2");

    // Macro toc
    TIMER_TOC;
}

// Calling timer inside recursive functions counts each iterations as a separate call
int fibonacci(int n){
    timer.tic("fibonacci_inside");
    static int x = 0;
    static int y = 1;

    if (n > 1){
        timer.toc("fibonacci_inside");
        return fibonacci(n-1);
    }else{
        timer.toc("fibonacci_inside");
        return 1;
    }
}

int main(){
    for (int i = 0; i < 2500; i++){
        std::vector<double> dummy_vec1(10000);
        efficientFuction(dummy_vec1);
    }

    parentFunction();

    // Here we can see the effect of the timer itself
    for (int i = 0; i < 10000; i++){
        timer.tic("test");
        timer.tic("dummy");
        timer.toc("dummy");
        timer.toc("test");
    }

    // Calling timer outside recursive functions only counts as one call
    timer.tic("fibonacci_outside");
    fibonacci(5);
    timer.toc("fibonacci_outside");

    // Here's a more advanced useage used to measure function call overhead
    for (int i = 0; i < 100000; i++){
        timer.tic("function_call_overhead");
        dummy(1, {2, 3});
        timer.toc("function_return_overhead");
    }

    timer.summary();
    return 0;
}
