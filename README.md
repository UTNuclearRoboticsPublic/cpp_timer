# C++ Timer Class
This repo contains a class useful for general C++ time benchmarking. All efforts have been made to reduce the effect of the timer class itself on the runtime performance of the code. All timer logic and processing is handled on a concurrent thread. Note that `CppTimer` is NOT thread safe, and should only be used from a single thread.

An example of an output from CppTimer is

~~~
============================= FUNCTION BREAKDOWN =============================

1: efficientFunction (2500): 673 us
______________________________________________________________________________

2: fibonacci_outside (1): 5 us
     |--- fibonacci_deep (1): 2 us
          |--- fibonacci_deep (2): 2 us
               |--- fibonacci_deep (4): 1 us
                    |--- fibonacci_deep (6): 973 ns
                         |--- fibonacci_deep (2): 203 ns
                         |--- Function Body: 770 ns
                    |--- Function Body: 789 ns
               |--- Function Body: 468 ns
          |--- Function Body: 251 ns
     |--- fibonacci_flat (15): 1 us
     |--- Function Body: 1 us
______________________________________________________________________________

3: function_call_overhead (100000): 17 ms
______________________________________________________________________________

4: function_return_overhead (100000): 12 ms
______________________________________________________________________________

5: reallyLongFunctionNameThatDoesn (1): 241 ns
______________________________________________________________________________

6: parentFunction (1): 1534 ms
     |--- childFunction1 (10000): 918 ms
          |--- loop_1 (10000): 444 ms
          |--- loop_2 (10000): 471 ms
          |--- Function Body: 2 ms
     |--- childFunction2 (100): 615 ms
          |--- childFunction1 (100): 9 ms
               |--- loop_1 (100): 4 ms
               |--- loop_2 (100): 4 ms
               |--- Function Body: 24 us
          |--- efficientFunction (100): 15 us
          |--- expensiveFunction (100): 602 ms
          |--- Function Body: 4 ms
     |--- Function Body: 546 us
______________________________________________________________________________


=================================== SUMMARY ===================================
                                Total Time   |  Times Called   |   Average Time
childFunction1:                     927 ms   |         10100   |          91 us
childFunction2:                     615 ms   |           100   |           6 ms
efficientFunction:                  689 us   |          2600   |         265 ns
fibonacci_outside:                    5 us   |             1   |           5 us
function_call_overhead:              17 ms   |        100000   |         179 ns
function_return_overhead:            12 ms   |        100000   |         126 ns
fibonacci_deep:                       2 us   |            15   |         165 ns
fibonacci_flat:                       1 us   |            15   |          92 ns
loop_1:                             449 ms   |         10100   |          44 us
loop_2:                             476 ms   |         10100   |          47 us
reallyLongFunctionNameThatDoesn:    241 ns   |             1   |         241 ns
expensiveFunction:                  602 ms   |           100   |           6 ms
parentFunction:                    1534 ms   |             1   |        1534 ms
~~~

# Installation
The cpp_timer package is a bare cmake package (not catkin) designed to be easy to make and install. It has no external dependencies outside of `boost`, which is likely already installed on your machine. If not, you can find the host page <a href = https://www.boost.org/ >here</a>.

To install the package, first clone or download the repository to your computer. From here on, we will assume that you are cloning using an ssh key as authentification. 

```bash
$ git clone git@github.com:UTNuclearRobotics/cpp_timer.git
```

Then navigate into the repo and create a build folder. From this folder we can invoke `cmake` to compile the library.

```bash
$ cd cpp_timer
$ mkdir build && cd build
$ cmake .. && make -j
```

Now that the library is compiled, you should be able to test it out by running the example executable from within the build folder

```bash
./example
```

Now if you wish to install the library so it can easily be used with other `cmake` or `catkin` projects, simply run the command

```bash
$ sudo make install 
```

# Usage
`CppTimer` takes inspiration from the MATLAB builtin functions tic and toc, which can be used to determine the duration of pieces of code. To begin using CppTimer (assuming you have already linked it to your CMake package using `find_package(cpp_timer)`), simply include the `cpp_timer/Timer.h` header and create an instance of the class. Once that is done, call the `tic` and `toc` methods with a section name to automatically get the duration between the function calls.

```c++
#include "cpp_timer/Timer.h"

int main(){
     cpp_timer::Timer timer;

     timer.tic("Example");
     // Put code to benchmark here
     timer.toc("Example");

     timer.summary();
}
```

In some cases, a function may have many return statements, and it can be troublesome to remember to put a corresponding `toc` before each return. To tackle this issue, `CppTimer` offers a scope based ticker, which automatically calls `tic` on construction and automatically calls `toc` on destruction.

```c++
int doSomething(int val){
     cpp_timer::Ticker t = timer.scopedTic("doSomething");

     if (val == 0) return -1;

     int error_val = runExpensiveFunction();

     if (error_val == 1)
          return -1;
     else if (error_val == 2)
          return 0;
     else
          return 1;
}
```

There are several macros defined in `cpp_timer/Timer.h` which allow for less wordy timer initializations. For all of these cases, you must first define a macro `TIMER_INSTANCE` to indicate which timer is responsible for the function calls. These macros include

* `TIMER_TIC` - Runs `tic` where the name of the calling function is automatically detected
* `TIMER_TOC` - Runs `toc` where the name of the calling function is automatically detected
* `STIC` - Stands for Scoped Tic. Creates and automatically names a `Ticker` object in the current scope

The header also defines `TIMER_PTIC`, `TIMER_PTOC`, and `SPTIC` for cases where `TIMER_INSTANCE` is a pointer type.

```c++
#include "cpp_timer/Timer.h"
#include "my_other_code.h"

static cpp_timer::Timer timer;
#define TIMER_INSTANCE timer

int func(){
     // Create a scoped Ticker object
     STIC;

     // Do some stuff
     return 0;
}

int main(){
     // Automatically call timer.tic("main")
     TIMER_TIC;

     // Do some stuff
     setup();
     func();
     cleanup()

     // Automatically call timer.toc("main")
     TIMER_TOC;

     return 0;
}

```

Finally, when all benchmarking is done, you can print the summary out to the terminal with the `summary()` function. This summary contains two sections. The first is a detailed breakdown of each function called according to its nested caller-callee relationship to other functions. Identical function calls from different parts of the code will appear separately in this section. The second section is a conglomerate summary of all functions called during the process runtime. Caller-callee relationships are ignored, and so identical function calls are merged together to give an overview of the independent function runtime perforances.

The summary function accepts two arguments. The first is the order in which to print the conglomerate summary. The second is the order in which to print the child function calls of each function being timed. These arguments are both of the type `cpp_timer::Timer::SummaryOrder` and are defined equivalent to

```c++
namespace cpp_timer::Timer{

     enum SummaryOrder{
          BY_NAME,
          BY_TOTAL,
          BY_AVERAGE,
          BY_CALL_COUNT,
          BY_CALL_ORDER
     };

}
```