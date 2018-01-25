# Author - Savio Pereira
# Last updated - 23rd Jan 2018

### Framework for a Singly Linked List to be used as an Adjacency List for a Graph:


## Step 1: Create a class for the node of a linked list: 
class LLNode: 
    
    # Constructor: 
    def __init__(self, data, weight = 0): 
        
        # Parameters of the node are: 
        # 1. Vertex index in the graph: 0 to numNodes
        self.vertex = data
        
        # 2. Weight of the edge : In case the node is being used for a graph:
        self.weight = weight
        
        # 3. Next node:
        self.next = None
        
### Step 2: Create a class for the Linked List: 

class LinkedList: 
    
    # Constructor: 
    def __init__(self): 
        self.head = None
        
        
    # Function to print the elements of the list:
    def print(self): 
        temp = self.head
        
        while (temp):
            
            print (temp.vertex ,":",temp.weight,  end =" -> ")
            
            temp = temp.next 
    
    # Function to insert an element at the head of the list: 
    def insert_head(self,new, weight =1): 
        
        # Create a node with the new data element
        NewNode = LLNode(new, weight)
        
        # Assign next pointer of New Node to current head
        NewNode.next = self.head
        
        # Make new node the head node of the list:
        self.head = NewNode
        
    # Function to remove the head node from the list: 
    def remove_head(self):
        
        # Assign the head node to a temp node: 
        temp = self.head
        
        # Shift the head node forward: 
        self.head = temp.next
        
        temp = None
        
    # Function to insert an element after a specific node in the list: 
    def insert(self,target, new): 
        
        inserted = False
        
        cursor = self.head
        
        # Loop over the list and find the target node: 
        while cursor is not None:
            
            if cursor.data == target:
                
                # Store the next node in a temporary variable: 
                temp = cursor.next
                
                # Create a new node to store the new element and assign it to the next pointer of cursor: 
                cursor.next = LLNode(new)
                
                # Assign the next pointer of new node to the temp node: 
                cursor.next.next = temp
                
                # Break out of the loop: 
                inserted = True
                 
                break 
            
            cursor = cursor.next
        
        # Check whether the element was inserted or not: 
        if (inserted): 
            
            print ("Successfully inserted New element")
        else: 
            
            print (" Unable to find target")
                
                
        
        # What are the end cases? : 
        # 1. What if there are two of the targte elements in the list?
        # 2. What if target and new are the same? 
        
    # Function to delete a specific node in the list: 
    def delete(self, target):
        
        cursor = self.head
        prev = None
        
        # Loop through the list to find the target element: 
        while cursor is not None:
            
            if (cursor.vertex == target): 
                
                # As long as the target is not the head node:
                if prev is not None:
                    prev.next = cursor.next
                
                # If the node to be deleted is the head node:
                else: 
                    self.head = cursor.next
                    
                
                cursor = None
                
                break
            
            # Move the previous node one step forward: 
            prev = cursor
            
            # Move cursor one step forward: 
            cursor = cursor.next
    
    
    # Function to reverse the elements of the list in place: 
    # Still have to analyse this algorithm better: Not understood it fully!!!
    def reverse(self): 
        
        # Loop through the list:
        
        # Keep track of 3 nodes: 
        # prev ,current , temporaryNextNode: 
        
        prev = None
        current = self.head
        
        # Loop through the list:
        while current is not None:
            
            # Extract the next node and store it in a temporary variable: 
            nextTemp = current.next
            
            # Switch the next pointer of current to point to the previous node: 
            current.next = prev
            
            # Move previous and current node ahead by one step for the next iteration: 
            prev = current
            current = nextTemp
            
        
        # At the end of the loop set the head of the list to be the previous node 
        self.head = prev
        
    
    # Function to return the kth node from the list: 
    def get_kth_node(self,k): 
        
        # Intialize a counter: 
        count = 1
        
        cursor = self.head
        
        while count <= k and cursor: 
            
            count +=1
            
            cursor = cursor.next
        
        return cursor