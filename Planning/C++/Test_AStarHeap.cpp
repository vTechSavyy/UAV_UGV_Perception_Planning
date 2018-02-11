/// Author: Savio Pereira

/// Date Created: 10th Feb 2018

/// Last Modified: 11th Feb 2018

/// Description: The purpose of this code is to test the implementation of the AStarHeap Class which will be used to
//               implement the Priority Queue in the A* algorithm.  The different functionality to be tested is:
//               1. The insert function - Check if new nodes are inserted in correct order and that the heap invariants are maintained after each insertion.
//               2. The update key function - Check that if a node that already exists in the heap is being inserted , then the update_key function should not insert a new copy
//                                            but only update the key of the existing node if it is less than the current key. Also check that the invariants of the heap are maintained
//               3. Extract min function - Check that the minimum element is extracted and that the heap invariants are maintanied


#include <iostream>
#include "AStarHeap.h"

using namespace std;

int main(int argc, char const *argv[]) {

  // Create an object of AStarHeap with Node elements:
  // Size of the grid associated with the Heap = 8: FOR TESTING ONLY!
  AStarHeap<Node>* testHeap = new AStarHeap<Node>(8);


  // Test 1: Test for the insert function:

  cout<<" --- Beginning Test #1 --- "<<endl;

  testHeap->insert(Node(0,8,2));
  testHeap->insert(Node(1,7,3));
  testHeap->insert(Node(2,6,1));
  testHeap->insert(Node(3,5,4));
  testHeap->insert(Node(4,4,4));

  testHeap->print();


  if (testHeap->check_invariants()){

    cout<<" Heap Inariants are maintained: -- Test #1 - PASSED!"<<endl;
  }
  else{

    cout<<" Heap Invariants are violated:   -- Test #1 - FAILED!"<<endl;

  }

  cout<<endl<<endl;


  // Test 2: Check the update_key function: Insert an existing node with a Lower key:

  cout<<" --- Beginning Test #2 --- "<<endl;


  testHeap->insert(Node(1,2,4));

  testHeap->print();

  if (testHeap->check_invariants() and testHeap->check_duplicates()){

    cout<<" Heap Invariants are maintained and NO duplicates: -- Test #2 - PASSED!"<<endl;
  }
  else{

    cout<<" Heap Invariants are violated or Duplicates present :   -- Test #2 - FAILED!"<<endl;

  }

  cout<<endl<<endl;


  // Test 3: Check the extract_min function:

  cout<<" --- Beginning Test #3 --- "<<endl;

  testHeap->extract_min();
  testHeap->print();

  if (testHeap->check_invariants()){

    cout<<" Heap Invariants are maintained: -- Test #3 - PASSED!"<<endl;
  }
  else{

    cout<<" Heap Invariants are violated:   -- Test #3 - FAILED!"<<endl;

  }


  cout<<endl<<endl;


  return 0;
}
