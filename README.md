# C++ Timer Class
This repo contains a class useful for general C++ time benchmarking
Note that this is not a high resolution timer; the class does not account
for it's own runtime (open a pull request with that feature if you'd 
like to add it), but it gives a good overview of the relative times spent
in each function

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

# Installing
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
