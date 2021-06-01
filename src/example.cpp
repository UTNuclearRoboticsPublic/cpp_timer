#include <iostream>
#include "cpp_timer/Timer.h"

static cpp_timer::Timer timer;

void expensiveFunction(std::vector<double> &vec){
    timer.tic("expensiveFunction");
    while(!vec.empty()){
        vec.erase(vec.begin());
    }
    timer.toc("expensiveFunction");
}

void efficientFuction(std::vector<double> &vec){
    timer.tic("efficientFunction");
    vec.clear();
    timer.toc("efficientFunction");
}

void childFunction1(){
    for (int i = 0; i < 10000; i++){
        int j = i/2.0;
    }
}

void childFunction2(){
    std::vector<double> dummy_vec2(10000);
    std::vector<double> dummy_vec3(10000);
    expensiveFunction(dummy_vec2);
    efficientFuction(dummy_vec3);
}

void parentFunction(){
    timer.tic("parentFunction");
    for (int i = 0; i < 100000; i++){
        timer.tic("childFunction1");
        childFunction1();
        timer.toc("childFunction1");
    }
    for (int i = 0; i < 100; i++){
        timer.tic("childFunction2");
        childFunction2();
        timer.toc("childFunction2");
    }
    timer.toc("parentFunction");
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

    timer.summary();
    return 0;
}
