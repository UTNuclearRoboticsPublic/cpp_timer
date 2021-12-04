# C++ Timer Class
This repo contains a class useful for general C++ time benchmarking
Note that this is not a high resolution timer; the class does not account
for it's own runtime (open a pull request with that feature if you'd 
like to add it), but it gives a good overview of the relative times spent
in each function

An example of an output from CppTimer is

~~~
===== FUNCTION BREAKDOWN =====
getCellOffsets (2): 3 ms

planCoveragePath (1): 3369 ms
     |--- bridge (11328): 1170 ms
          |--- bridgeFast (6788): 5 ms
          |--- connectSingleGroup (8206): 730 ms
          |--- getValidSurroundingCells (9016): 76 ms
          |--- partition (1956): 157 ms
          |--- other: 202 ms
     |--- migrate (2797): 3 ms
     |--- partition (1): 0 ms
     |--- planDirectPath (166): 2159 ms
          |--- bridge (385421): 877 ms
               |--- bridgeFast (2412): 2 ms
               |--- connectSingleGroup (2526): 291 ms
               |--- getValidSurroundingCells (2998): 27 ms
               |--- partition (1432): 124 ms
               |--- other: 433 ms
          |--- migrate (385421): 405 ms
          |--- other: 877 ms
     |--- other: 37 ms

populatePointCloudObstacles (1): 1306 ms

===== SUMMARY =====
                                Total Time   |  Times Called   |   Average Time
bridge:                            2047 ms   |        396749   |          5 us
bridgeFast:                           7 ms   |          9200   |        855 ns
connectSingleGroup:                1021 ms   |         10732   |         95 us
getCellOffsets:                       3 ms   |             2   |          1 ms
getValidSurroundingCells:           103 ms   |         12014   |          8 us
migrate:                            409 ms   |        388218   |          1 us
partition:                          282 ms   |          3389   |         83 us
planCoveragePath:                  3369 ms   |             1   |       3369 ms
planDirectPath:                    2159 ms   |           166   |         13 ms
populatePointCloudObstacles:       1306 ms   |             1   |       1306 ms
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
$ cmake .. & make -j
```

Now that the library is compiled, you should be able to test it out by running the example executable from within the build folder

```bash
./example
```

Now if you wish to install the library so it can easily be used with other `cmake` or `catkin` projects, simply run the command

```bash
$ sudo make install 
```