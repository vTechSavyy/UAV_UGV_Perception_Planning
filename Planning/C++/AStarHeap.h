#ifndef A_STAR_HEAP_H
#define A_STAR_HEAP_H

/// Author: Savio Pereira

/// Date Created: 9th Feb 2018

/// Last Modified: 10th Feb 2018

// Description: Templated class for implementing a binary Min-heap specifically to be used as the Priority Queue in the A* algorithm.
// The time complexity for the Priority Queue using a Heap is:
// 1. insert - O(log n) - Worst case
// 2. extract_min - O (log n) - Worst case

// This is better than just using a simple list (array) is because the time to search for the element in the array with the lowest key is O(n):

// The implementation of this Heap uses a STL C++ vector to store the elements of the Heap:

// Elements are of the type : 'Node' - Implemented in the 'Node' class and included in this file:

// The Heap can be implemented using pointers as well but requires keeping track of the parent of each element as well whereas with a dynamic array all we need is a mapping of the indices from parent
// to left and right children

// Invariants of the Binary Min-Heap are:
// 1. An element in the heap is always less than both is left and right children
// 2. If an element is at index 'i' , its left child is at index : (2*i + 1)
// 3. If an element is at index 'i' , its right child is at index : (2*i + 2)


#include "Node.h"
#include <vector>
#include <iostream>
#include <unordered_set>

using namespace std;

// Declaration of the class:
template <class Elem>
class AStarHeap {


protected:

  // Parameters of the AStarHeap class:

  // 1. heapArray - The dynamic array used for storing the elements of the Heap: Dynamic array implemeted using c++ STL vector class:
  vector<Elem> heapArray;

  // 2. indArray - It is a dynamic array (implemented using STL vector<int>) that stores the heapArray indices of all the points in the grid.
  vector<int> indArray;

  // 3. Index of the last element in the Heap array:
  int lastIdx;


public:

  // 0. Constructor:
  AStarHeap(int numNodesInGrid = 0) {

    // Set the last index to -1:
    lastIdx = -1;

    // Intialize all the elements in 'indArray' to -1:
    for (int i = 0; i < numNodesInGrid; i++){
      indArray.push_back(-1);
    }


  }

  // 1. Function to swap two elements in the heap array given their indices:
  void swap(int idx1, int idx2);


  // 2. Function to insert an element into the heap and yet maintain the invaraints of the Heap:
  void insert(Elem newElem);

  // 3. Bubble up Function: [Recursively called]
  // Description: Given the index of an element in the heap array (heapIdx), bubble it up until it is greater than its parent:
  void bubble_up(int heapIdx);


  // 4. Bubble down function: [Recursively Called]:
  // Description: Given the index of an element in the heapArray(heapIdx), bubble it down until it is less than its children:
  void bubble_down(int heapIdx);

  // 5. Function to extract the minimum element from the Heap: At the same time maintain the invariants of the heap:
  // Returns: The minimum element in the Heap as type 'Elem'
  Elem extract_min();

  // 6 . Function to update the key and bestNeighbor of an element in the Heap:
  void update_key(Elem newElem);

  // 7. Function to print the elements of the heap: ONLY FOR TESTING PURPOSES!
  void print();

  // 8. Function to check the invariants of the Heap at any given point: ONLY FOR TESTING PURPOSES!
  // Returns: A boolean flag that is True when  the heap invariants are maintained:
  bool check_invariants();

  // 9. Function to check duplicity of nodes in the Heap: ONLY FOR TESTING PURPOSES!
  // Returns: A boolean flag that is True when the heap DOES NOT contain duplicates
  bool check_duplicates();


};  // END class AStarHeap


// Include the implementation file here: Temporary Workaround:
#include "AStarHeap.cpp"


#endif
