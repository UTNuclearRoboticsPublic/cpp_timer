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
planCoveragePath (1): 3351 ms
     |--- bridge (11328): 1177 ms
          |--- bridgeFast (6788): 5 ms
          |--- connectSingleGroup (8206): 735 ms
          |--- getValidSurroundingCells (9016): 77 ms
          |--- partition (1956): 157 ms
          |--- other: 203 ms
     |--- migrate (2797): 3 ms
     |--- partition (1): 0 ms
     |--- planDirectPath (166): 2134 ms
          |--- bridge (385421): 868 ms
               |--- bridgeFast (2412): 2 ms
               |--- connectSingleGroup (2526): 287 ms
               |--- getValidSurroundingCells (2998): 27 ms
               |--- partition (1432): 124 ms
               |--- other: 428 ms
          |--- migrate (385421): 398 ms
          |--- other: 868 ms
     |--- other: 37 ms
populatePointCloudObstacles (1): 1312 ms

===== SUMMARY =====
                                Total Time   |  Times Called   |   Average Time
bridge:                            2046 ms   |        396749   |          5 us
bridgeFast:                           8 ms   |          9200   |          0 us
connectSingleGroup:                1023 ms   |         10732   |         95 us
getCellOffsets:                       3 ms   |             2   |       1877 us
getValidSurroundingCells:           104 ms   |         12014   |          8 us
migrate:                            402 ms   |        388218   |          1 us
partition:                          282 ms   |          3389   |         83 us
planCoveragePath:                  3351 ms   |             1   |       3351 ms
planDirectPath:                    2134 ms   |           166   |         12 ms
populatePointCloudObstacles:       1312 ms   |             1   |       1312 ms
~~~
