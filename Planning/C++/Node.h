#ifndef NODE_H
#define NODE_H

/// Author: Savio Pereira

/// Date Created: 9th Feb 2018

/// Last Modified: 9th Feb 2018

/// Description: Class to implement a node to be used in the Binary Min-Heap which in turn will be used as the Priority Queue in A* algorithm:

/// The node stores three parameters:

// 1. nodeNum - The index of the node in the grid on which A* is being run.
// 2. key - Key is the distance of the node from the start (a.k.a cost-to-come). It is used as the key in the Priority Queue
// 3. bestNeighbor - The index of the node which is the 'bestNeighbor' of the current node so that the shortest path can be back traced once we reach the goal:

class Node {

public:

  int nodeNum;
  float key;
  int bestNeighbor;

  // Constructor:
  Node(int nodeNum, float key = 0, int bestNeighbor = -1){

    this->nodeNum = nodeNum;
    this->key = key;
    this->bestNeighbor = bestNeighbor;
  }

  // Overload the less than operator for the node since it will be used as an element in the Heap:
  friend bool operator < (const Node &n1 , const Node &n2);


  // Overload the greater than operator for the node since it will be used as an element in the Heap:
  friend bool operator > (const Node &n1 , const Node &n2);


};


// Implementation of the overloaded operators:
// 1. Less than operator:
bool operator < (const Node &n1 , const Node &n2){

  return n1.key < n2.key;

}


// 2. Greater than operator:
bool operator > (const Node &n1 , const Node &n2){

  return n1.key > n2.key;

}


#endif
