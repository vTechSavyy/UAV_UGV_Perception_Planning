### Desription: Script to implement the Heap Class specifically for Dijkstra algorithm: 
# dHeap - Stands for Dijkstra Heap

# Last updated: 25th Jan 2018: 

## Logic: This implementation supports the additional functionality of decreaseKey of the element. This is achieved by maintaining 
#         a mapping between vertices in the graph and their indices in the heap list(array)

import numpy as np
from Heap import Heap

## Step 1: Create a class for the node in the dijkstra heap: 
class dHeapNode(object): 
    
    # Constructor: 
    # The node has two parameters: 
    # 1. vertex - Index of the vertex in the graph
    # 2. key -    Key used in sorting the heap (In this case distance from the start node).
    # 3. bestNeighbor - For back tracking and calculating the shortest path
    def __init__(self, vertex, key, bestNeighbor): 
        
        self.vertex = vertex
        self.key = key
        self.bestNeighbor = bestNeighbor
        
        
    # Comparison operators: Comparison will be based on the keys: 
    # 1. Less than:
    def __lt__(self,other): 
        
        return self.key < other.key
    
    # 2. Greater than: 
    def __gt__(self,other):
        
        return self.key > other.key
    
    
## Step 2: Create the class for Dijkstra Heap: Inherit from the regular Heap class 
class dHeap(Heap):    
    
    # Constructor: 
    # One additional parameter: 
    # 1. indList - List of indices of each vertex in the graph: 
    def __init__(self,numNodes=0): 
        
        # Call the constructor of the base class :       
        Heap.__init__(self)
        
        
        # Initialize the indices of all vertices in the Graph to Infinity:
        self.indList = []
        
        for idx in range(numNodes):
            self.indList.append(np.Inf)
            
            
    # 0. Swap the elements at two indices of the heapList: 
    def swap(self,idx1,idx2):
        
        # Extract the nodes so that later their indices in the Heap list can also be modified
        node1 = self.heapList[idx1]
        node2 = self.heapList[idx2]
        
        # Typical swap operation: 
        temp = self.heapList[idx1]
        self.heapList[idx1] = self.heapList[idx2]
        self.heapList[idx2]= temp
        
        # Modify the indices of the vertices in the Heap List
        self.indList[node1.vertex] = idx2
        self.indList[node2.vertex] = idx1
            
    # 2. Function to insert new node: 
    def insert(self, target):
        
        # Speical case: If the Heap is empty: 
        if not self.heapList: 
            
            self.heapList.append(target)
            
            self.currMaxIdx =0
            
            self.indList[target.vertex] = self.currMaxIdx
        
        # If the target vertex is NOT already in the heap:
        elif self.indList[target.vertex] == np.Inf: 
            
            # Add the new node to the heap:
            self.heapList.append(target) 
            
            # Update the heap counter:
            self.currMaxIdx +=1
            
            self.indList[target.vertex] = self.currMaxIdx
            
            # Now repair the heap by using the Bubble-up operation: 
            self.bubble_up(self.currMaxIdx)
            
         
        # If the target vertex is already in the heap:
        else: 
            
            # Doon't add a copy of the new node. Simply update its key
            self.updateKey(target)
            
            
    # 3. Function to decrease the key of a certain vertex in the graph:
    # Description: target is a dHeapNode that needs to be updated
    def updateKey(self, target): 
                
        # Update the node in the heap only if new key is less than current key:        
        if target < self.heapList[self.indList[target.vertex]]: 
            
            # Decrement the key if it is less: 
            self.heapList[self.indList[target.vertex]].key = target.key
            
            # Update the best neighbor of the current node: 
            self.heapList[self.indList[target.vertex]].bestNeighbor = target.bestNeighbor
            
            # Since the key has been decremented bubble up the current node:
            self.bubble_up(self.indList[target.vertex])
            

            
    # 4. Overiding the function to print the nodes in the dHeap: 
    def print(self): 
        
        for node in self.heapList: 
            
            print ( node.vertex, "[", node.key,"]", end="--> ")
        
        print (" ")
        
        for idx in self.indList: 
            
            print (idx , end="-> ")
            
        print (" ")
        
    # 5. Extract min operation: Function to remove the min-element from the heap but still maintain the heap structure: 
    def extract_min(self): 
        
        # Remove the first index element to a temporary storage area: 
        minNode = self.heapList[0]
        
        # Swap the last element with the first element of the heapList: 
        self.swap(0,self.currMaxIdx)
        
        # Remove the element at the last index: 
        self.heapList.pop(self.currMaxIdx)
        
        #  Update the Heap index for the popped element: Change it back to np.Inf
        self.indList[minNode.vertex] = np.Inf
        
        # Decrement the current max index by 1: 
        self.currMaxIdx -= 1
        
        # Repair the heap by bubbling down the top most element: 
        self.bubble_down(0)
        
        # Return the min element: 
        return minNode
    
            
