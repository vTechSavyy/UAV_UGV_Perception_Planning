### Desription: Script to implement the Heap Class for generic applications: 

# Last updated: 23rd Jan 2018: 

# Invariants of a Binary Heap Stored as a dynamic array (List in Python) : 
# Parent to children: 
# If parent == idx: -> Left child = (2*idx + 1) -- Right child = (2*idx +2)
# Children to parent: 
# If child == idx:  -> Parent = floor((idx-1)/2)

class Heap: 
    
    # Constructor: 
    # Parameters: 
    # 1. heapList - A list which stores the nodes. The list size can change dynamically
    # 2. currMaxIdx - The index of the last element in the Heap. 
    def __init__(self):
        
        # List (array) to store elements of the heap:
        self.heapList = []
        
        # Heap counter:
        self.currMaxIdx = None
        
    
    # Operations: 
    # 0. Swap the elements at two indices of the heapList: 
    def swap(self,idx1,idx2):
               
        # Typical swap operation: 
        temp = self.heapList[idx1]
        self.heapList[idx1] = self.heapList[idx2]
        self.heapList[idx2]= temp
       
        
        
    # 1. Bubble up operation: Given a heap index, this function bubbles up the element at that index based on its key. 
    def bubble_up(self, heapIdx): 
        
        # Get parent index: 
        parentIdx = int((heapIdx-1)/2)
        
        # Check if current element is less than its parent:
        if self.heapList[heapIdx] < self.heapList[parentIdx]: 
            
            # Swap the node with its parent: 
            self.swap(heapIdx, parentIdx)
            
            # Recursively call the bubble-up function until the node reaches its correct place: 
            self.bubble_up(parentIdx)
            
        else: 
            
            return
        
    # 2. Bubble down operation: Given a heap index, this function bubbles down the element at that index based on its key
    def bubble_down(self, heapIdx): 
        
        # Get the indices of left and right child: 
        left =  heapIdx*2 + 1
        right = heapIdx*2 + 2
        
        # Check whether left index is greater than current max index: 
        if left > self.currMaxIdx:             
            return
        
        # Left index is equal to current max but right is greater:
        elif right > self.currMaxIdx: 
            
            # Compare current node with left child: If greater then:
            if self.heapList[heapIdx] > self.heapList[left]: 
                
                # Swap the current node with left child:
                self.swap(heapIdx,left)
                
                # Recursibely bubble down the left sub-tree: 
                self.bubble_down(left)
                
            return 
            
        # Both left and right are less than or equal to current max index: 
        else: 
            # Compare current node with left child: If greater then:
            if self.heapList[heapIdx] > self.heapList[left]: 

                # If left node is less than right node: 
                if self.heapList[left] < self.heapList[right]:

                    # Swap the current node with left child:
                    self.swap(heapIdx,left)

                    # Recursibely bubble down the left sub-tree: 
                    self.bubble_down(left)

                # If right node is less than left node meaning that it is also less than current node: 
                else: 

                     # Swap the current node with right child:
                    self.swap(heapIdx,right)

                    # Recursibely bubble down the right sub-tree: 
                    self.bubble_down(right)


            # Compare current node with right child: If greater then:
            if self.heapList[heapIdx] > self.heapList[right]: 

                # If right node is less than left node: 
                if self.heapList[right] < self.heapList[left]:

                    # Swap the current node with right child:
                    self.swap(heapIdx,right)

                    # Recursibely bubble down the right sub-tree: 
                    self.bubble_down(right)

                # If left node is less than right node meaning that it is also less than current node: 
                else: 

                     # Swap the current node with left child:
                    self.swap(heapIdx,left)

                    # Recursibely bubble down the left sub-tree: 
                    self.bubble_down(left)


            # If current node is less than both left and right child then return: 
            return              
                
                
        
        
    # 3. Insert: Function that inserts a node in the Heap and maintains its structure: 
    def insert(self, target):
        
        # Speical case: If the Heap is empty: 
        if not self.heapList: 
            
            self.heapList.append(target)
            
            self.currMaxIdx =0
        
        # General case: 
        else: 
            
            self.heapList.append(target) 
            
            self.currMaxIdx +=1
            
            # Now repair the heap by using the Bubble-up operation: 
            self.bubble_up(self.currMaxIdx)
            
    # 4. Extract min operation: Function to remove the min-element from the heap but still maintain the heap structure: 
    def extract_min(self): 
        
        # Remove the first index element to a temporary storage area: 
        minNode = self.heapList[0]
        
        # Swap the last element with the first element of the heapList: 
        self.swap(0,self.currMaxIdx)
        
        # Remove the element at the last index: 
        self.heapList.pop(self.currMaxIdx)
        
        # Decrement the current max index by 1: 
        self.currMaxIdx -= 1
        
        # Repair the heap by bubbling down the top most element: 
        self.bubble_down(0)
        
        # Return the min element: 
        return minNode
    
    # 5. Function to print the Heap: 
    def print(self): 
        
        print (self.heapList)
        
        