# -------------------------------------------------- #

For BOSCH automated driving Team :

C++ code sample by  Savio Pereira


# -------------------------------------------------- #

Description: The code in this folder is a utility for the A* planning algorithm. The planning algorithm maintains a priority queue
             of vertices in the grid/graph that are being visited and at each step selects the element in the queue with the lowest key. A naive array implementation
             of this queue requires a search time that is O (n) while the insertion time is O(1). However, when implemented using a Heap data structure the search time is
             O(log n) and the insertion time is O (log n).
             This sample code provides the utilities to implement the Priority Queue using a Heap Data structure.

# -------------------------------------------------- #

Files:
# 1. Node.h -            Header file containing Class for implementing the Nodes that will be used in the Heap
# 2. AStarHeap.h -       Header file containing templated Class for Heap to be used as the Priority Queue. The heap is implemented using C++ STL vector
# 3. AStarHeap.cpp -     Implementation file for the templated Heap Class
# 4. TestAStarHeap.cpp - Utility file for testing the implementation of the Heap as a Priority Queue.


# ------------------------------------------------- #
Build and Run Commands:

Uses GNU G++ compiler:

# 1. Compilation command:
g++ -Wall -std=c++11 -o testHeap TestAStarHeap.cpp

# 2. Run command:
./testHeap
