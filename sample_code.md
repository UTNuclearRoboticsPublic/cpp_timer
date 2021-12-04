# Sample Code

The Timer class operates under a very simple premise. Taking inspiration from Matlab's timer functions, one only needs a "tic" and a "toc" to measure a duration. These times are associated with a mandatory label, which is often the name of the function that you are timing, but it can really be anything. Times with the same label will be lumped together even if they occur in different parts of the code, but their separate runtimes will also be available for viewing in the Timer summary.

Let's go over a simplistic example. The first thing we're going to need is a Timer class instance. If you're keeping it to a simple one file program, you can declare it `static` like this

```cpp
// Create a timer instance
static cpp_timer::Timer timer;
```

Additionally, if we want to streamline things later on with this package's macros, we can define a timer instance macro. The name of the timer must match the name of our `static cpp_timer::Timer` from the last step

```cpp
// Define timer instance macro
#define TIMER_INSTANCE timer
```

Now we're going to write a bunch of functions of various efficiencies:

```cpp
// Please never actually write code like this
void expensiveFunction(std::vector<double> &vec){
    while(!vec.empty()){
        vec.erase(vec.begin());
    }
}

// This is how you would do it in practice
void efficientFuction(std::vector<double> &vec){
    vec.clear();
}

// This function will be called from within another function
void childFunction1(){
    TIMER_TIC;
    for (int i = 0; i < 10000; i++){
        int j = i/2.0;
    }
    TIMER_TOC;
}

// This function will also be called from within another function
void childFunction2(){
    std::vector<double> dummy_vec2(10000);
    timer.tic("expensiveFunction");
    expensiveFunction(dummy_vec2);
    timer.toc("expensiveFunction");
}

// This is where those two functions are called from
void parentFunction(){
    for (int i = 0; i < 100000; i++){
        childFunction1();
    }
    for (int i = 0; i < 100; i++){
        timer.tic("childFunction2");
        childFunction2();
        timer.toc("childFunction2");
    }
}
```

There are a couple things to note about this code. Note how for every function that we wanted to time, we included a `tic` and a `toc` call. Additionally, we provide the name of the timer label for each function call. If you want to, you can add a separate timer for a section of code, such as an expensive `for` loop with a label of your choosing. Also, note that in `childFunction1` we used the convenience macros `TIMER_TIC` and `TIMER_TOC`. These automatically determine the name of the parent function using the `BOOST_PRETTY_FUNCTION` macro.

 Generally speaking, you want to nest these as closely to the functions as possible. In general, putting it at the start and end of a function is easiest, but one must be careful of functions with multiple `return` statements. It is also acceptable to put it before and after the function call, as this will capture the function call overhead as well. Note that an unmatched tic will cause the function to throw a failed assertion error.

Now finally we write our `main` function

```cpp
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
```

Since we placed all of the `tic` and `toc` calls within the functions themselves, this main function can be written fairly cleanly. Additionally, we added an extra section to test the average runtime of the timer itself. The reason for having two different sets of timer tics in the `for` loop is that the timer starts timing at the last line of the `tic` function and at the first line of the `toc` function, so just using the "test" label would not have given an accurate measurement.

The output of the Timer gives us a summary of the runtime of all functions measured. The initial function breakdown gives the relational tree connecting the functions. The numbers in parentheses are the number of times that function was called. Every timer layer has a field called "other" which is the time not spent inside of a nested function.

```
===== FUNCTION BREAKDOWN =====
efficientFunction (2500): 0 ms

parentFunction (1): 5131 ms
     |--- childFunction1 (100000): 4361 ms
     |--- childFunction2 (100): 725 ms
          |--- efficientFunction (100): 0 ms
          |--- expensiveFunction (100): 721 ms
          |--- other: 4 ms
     |--- other: 45 ms

test (10000): 5 ms
     |--- dummy (10000): 0 ms
     |--- other: 5 ms

===== SUMMARY =====
                                Total Time   |  Times Called   |   Average Time
childFunction1:                    4361 ms   |        100000   |         43 us
childFunction2:                     725 ms   |           100   |          7 ms
dummy:                              429 us   |         10000   |         42 ns
efficientFunction:                  216 us   |          2600   |         83 ns
expensiveFunction:                  721 ms   |           100   |          7 ms
parentFunction:                    5131 ms   |             1   |        836 ms
test:                                 5 ms   |         10000   |        543 ns
```