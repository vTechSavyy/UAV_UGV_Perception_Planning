/// Author: Savio Pereira

/// Date Created: 9th Feb 2018

/// Last Modified: 10th Feb 2018

/// Description: Implementation file for the Binary Min-Heap to be uses as a Priority Queue in A * algorithm:

  // 1. Implementation: Function to swap two elements in the heap array given their indices and also to update their indices in the Heap Array:
  template <class Elem>
  void AStarHeap<Elem>::swap(int idx1, int idx2){

    // Extract the nodes so that later their indices in heapArray can also be modified
    Node node1 = heapArray[idx1];
    Node node2 = heapArray[idx2];

    // Swap the two elements here:
    Elem temp = heapArray[idx1];
    heapArray[idx1] = heapArray[idx2];
    heapArray[idx2] = temp;

    // Modify the indices of the vertices in the heapArray
    indArray[node1.nodeNum] = idx2;
    indArray[node2.nodeNum] = idx1;

  }


  // 2. Implementation: Bubble up Function: [Recursively called]
  // Description: Given the index of an element in the heap array (heapIdx), bubble it up until it is greater than its parent:
  template <class Elem>
  void AStarHeap<Elem>::bubble_up(int heapIdx){

    // Get the index of the parent element:
    int parentIdx = (int) (heapIdx - 1)/2;

    // Failsafe:
    if (parentIdx <0){
      return;
    }

    // Check to see if the element at 'heapIdx' is less than its parent:
    if (heapArray[heapIdx] < heapArray[parentIdx]){

      // Swap the element at 'heapIdx' with its parent:
      swap(heapIdx, parentIdx);

      // Recursively keep doing this until the element is greater than its parent:
      bubble_up(parentIdx);

    }

    // Termination case for the recursive function:
    else{

      return;
    }

  }

  // 3. Implementation: Bubble down function: [Recursively Called]:
  template <class Elem>
  void AStarHeap<Elem>::bubble_down(int heapIdx){

    // Get the indices of left and right children:
    int leftIdx  = heapIdx*2 + 1;
    int rightIdx = heapIdx*2 + 2;

    // Check to see if the left child index exceeds the index of the last element: If yes then return out since the element cannot be further bubble_down:
    if (leftIdx > lastIdx){
      return;
    }

    // The case if the leftIdx is equal to lastIdx and thus only rightIdx is greater than lastIdx:
    else if (rightIdx > lastIdx){

      // Compare the element at heapIdx only with the left child and if it is greter then swap it:
      if (heapArray[heapIdx] > heapArray[leftIdx]){

        swap(heapIdx, leftIdx);

      }

    }

    // The case where both leftIdx and rightIdx are less than lastIdx:
    // There are two cases to be checked for here:
    // a. Element at heapIdx is greater than left child:
    // or
    // b. Element at heapIdx is greater than right child:
    else {

      // Check if Element at heapIdx is greater than left or right child:
      if ( heapArray[heapIdx] > heapArray[leftIdx] or heapArray[heapIdx] > heapArray[rightIdx]){

        // Decide which child the element needs to be swapped with:
        // If left child is less than right child:
        if (heapArray[leftIdx] < heapArray[rightIdx]){

          // Swap the element at heapIdx with its left child:
          swap(heapIdx, leftIdx);

          // Recrusively bubble down the left sub-heap:
          bubble_down(leftIdx);
        }

        // If right child is less than or equal to left child:
        else{

          // Swap the element with its right child:
          swap(heapIdx, rightIdx);

          // Recrusively bubble down the right sub-tree:
          bubble_down(rightIdx);

        }

      } // END if heapArray[heapIdx] > heapArray[leftIdx] or heapArray[heapIdx] > heapArray[rightIdx]

      // If element at heapIdx is less than or equal to both left and right children then return out of the function:
      else{

        return;

      }

    } // END else (Both leftIdx and rightIdx are less than lastIdx)

  } //END bubble_down function:




  // 4. Implementation: Function to insert an element (of type 'Elem') into the heap and yet maintain the invariants of the heap: For AStarHeap:
  template <class Elem>
  void AStarHeap<Elem>::insert(Elem newElem){


      // Check if the new element is not yet in the Heap:
      if (indArray[newElem.nodeNum] == -1){

          // Add the new element to the empty heapArray:
          heapArray.push_back(newElem);

          // Increment the lastIdx of the heap:
          lastIdx++;

          // Update 'indArray':
          indArray[newElem.nodeNum] = lastIdx;

          // Ensure that the invariants of the heap remain true. This is done by bubbling up the new element:
          // Worst case: O(log n) operation:
          bubble_up(lastIdx);



    }

    // If it already exists in the Heap, then simply update its key and bestNeighbor:
    else{

      update_key(newElem);

    }

  } // END function insert()


  // 5. Function to extract the minimum element from the Heap: At the same time maintain the invariants of the heap:
  // Returns: The minimum element in the Heap as type 'Elem'
  template <class Elem>
  Elem AStarHeap<Elem>::extract_min(){

    // Temporary storage for the minimum element
    Elem minElem = heapArray[0];

    // Swap the last element of the heapArray with first element:
    swap(0, lastIdx);

    // Pop off the element at lastIdx:
    heapArray.pop_back();

    // Update indArray for the popped element: Change it back to -1:
    indArray[minElem.nodeNum] = -1;

    // Decrement the lastIdx of the heapArray:
    lastIdx--;

    // Maintain the heap invaraints by bubbling down the current first element: Worst case O(log n) operation
    bubble_down(0);

    // Return the minimum element:
    return minElem;

  }

  // 6. Function to update the key and bestNeighbor of an element in the Heap:
  template <class Elem>
  void AStarHeap<Elem>::update_key(Elem newElem){

    // Update the element in the heap only if new key is less than current key:
    if (newElem < heapArray[indArray[newElem.nodeNum]]){

      // Update the key:
      heapArray[indArray[newElem.nodeNum]].key = newElem.key;

      // Update the best neighbor of the current element:
      heapArray[indArray[newElem.nodeNum]].bestNeighbor = newElem.bestNeighbor;

      // Since the key has been decremented bubble up the current element:
      bubble_up(indArray[newElem.nodeNum]);

    }

    return;
  }

  // 7. Function to print the elements of the heap: ONLY FOR TESTING PURPOSES!
  template <class Elem>
  void AStarHeap<Elem>::print(){

    cout<<" The elements in the Heap are: ' Node_Number [key] ' "<<endl;

    for (typename vector<Elem>::iterator heapIt = heapArray.begin(); heapIt != heapArray.end(); ++heapIt){

      cout<<"-->"<<heapIt->nodeNum << ":[" << heapIt->key <<"]";

    }

    cout<<endl;
    cout<<"  ------------------------------- " <<endl;
  }


  // 8. Function to check the invariants of the Heap at any given point: ONLY FOR TESTING PURPOSES!
  // Returns: A boolean flag that is True when  the heap invariants are maintained:
  template <class Elem>
  bool AStarHeap<Elem>::check_invariants(){

    // Check that each node has a key that is less than its left and right child:
    for (int idx = 0; idx <= lastIdx; idx++){

      int leftIdx = (2*idx + 1);
      int rightIdx = (2*idx + 2);

      // Check left child:
      if (leftIdx <= lastIdx){

          if (heapArray[leftIdx].key < heapArray[idx].key){

            return false;

          }
      }

      // Check right child:
      if (rightIdx <= lastIdx){

          if (heapArray[rightIdx].key < heapArray[idx].key){

            return false;

          }
      }

    }  // END for loop

    // If all cases are passed then return True:
    return true;

  }

  // 9. Function to check duplicity of nodes in the Heap: ONLY FOR TESTING PURPOSES!
  // Returns: A boolean flag that is True when the heap DOES NOT contain duplicates
  template <class Elem>
  bool AStarHeap<Elem>::check_duplicates(){

    // Create an unordered_set to store the node numbers (ints) of elements of the Heap: Search time for unordered_set is O(1)
    unordered_set<int> dupCheckSet;

    // Iterate through the Heap array:
    for (typename vector<Elem>::iterator heapIt = heapArray.begin(); heapIt != heapArray.end(); ++heapIt){

      // If the node number does not exists in the dupCheckSet:
      if (dupCheckSet.find(heapIt->nodeNum) == dupCheckSet.end()){

        // Add it to the set:
        dupCheckSet.insert(heapIt->nodeNum);
      }

      // If it does not exist then return out with false:
      else{

        return false;
      }

    }

    // After all checks have passed return out with true:
    return true;


  }
