### Utility Functions for Planning algorithms:
import params
import numpy as np
from dHeap import dHeap
from dHeap import dHeapNode

# Description : This function returns two arrays:
# 1. X_grid_re - Contains the X-cordinates of all the nodes:
# 2. Y_grid_re - Contains the Y-cordinates of all the nodes:
def generate_grid():
    # 1-D vector in X -direction:
    x_array = np.arange(0,params.widthX + params.gridRes, params.gridRes)

    # 1-D vector in Y -direction:
    y_array = np.arange(0,params.widthY + params.gridRes, params.gridRes)

    # Create the X-Y meshgrid:
    X_grid, Y_grid = np.meshgrid(x_array, y_array)

    rows, cols = X_grid.shape

    # Reshape the grid arrays:
    X_grid_re = np.reshape(X_grid, (rows*cols,-1))
    Y_grid_re = np.reshape(Y_grid, (rows*cols,-1))

    # Return the reshaped grids with X and Y co-ods:
    return X_grid_re, Y_grid_re


# Description : This function returns two lists with the indices of the neighbors of the given node (index).
# The first list dNeighbors contains all the diagonal neighbors, pNeighbors contains all perpendicular neighbors
def get_neighbor_indices(nodeIdx):

    #global params.widthX, params.widthY, params.gridRes, X,Y


    # TO DO: Draw a diagram depicting the co-ordinate systems:


    # First get X and Y co-ordinates from nodeIdx:
    nodeX = params.X[nodeIdx]
    nodeY = params.Y[nodeIdx]


    # Compute number of nodes in one row:
    idxWidth = int(params.widthX/params.gridRes) + 1


    ### a.Special Border cases:

    # 1. Four Corners: 3 neighbors
    # 1.a Top left corner
    if nodeX ==0 and nodeY==0:

        # Depict with diagram?
        pNeighbors = [nodeIdx +1, nodeIdx +idxWidth]
        dNeighbors = [nodeIdx+idxWidth+1]

        return pNeighbors , dNeighbors

    # 1.b Top right corner
    if nodeX == params.widthX and nodeY ==0:

        pNeighbors  = [nodeIdx -1 , nodeIdx + idxWidth]
        dNeighbors =  [nodeIdx + idxWidth -1]

        return pNeighbors , dNeighbors

    # 1.c Bottom right corner
    if nodeX ==params.widthX and nodeY == params.widthY:

        pNeighbors = [nodeIdx -1, nodeIdx - idxWidth]

        dNeighbors = [nodeIdx - idxWidth -1]

        return pNeighbors , dNeighbors

    # 1.d Bottom left corner
    if nodeX ==0 and nodeY == params.widthY:

        pNeighbors = [nodeIdx +1, nodeIdx - idxWidth]
        dNeighbors = [nodeIdx - idxWidth +1]

        return pNeighbors , dNeighbors



    # 2. Four edges: 5 neighbors
    # 2.a Top edge
    if nodeY ==0:

        pNeighbors = [nodeIdx - 1, nodeIdx +1 , nodeIdx + idxWidth  ]
        dNeighbors = [nodeIdx + idxWidth -1 , nodeIdx + idxWidth +1]

        return pNeighbors , dNeighbors


    # 2.b Right edge:
    if nodeX == params.widthX:


        pNeighbors = [ nodeIdx - idxWidth, nodeIdx -1, nodeIdx + idxWidth]
        dNeighbors = [nodeIdx - idxWidth -1 , nodeIdx + idxWidth -1]

        return pNeighbors , dNeighbors

    # 2.c Bottom edge:
    if nodeY == params.widthY:

        pNeighbors = [nodeIdx-1, nodeIdx+1,  nodeIdx - idxWidth]
        dNeighbors = [nodeIdx -idxWidth -1, nodeIdx -idxWidth+1]

        return pNeighbors , dNeighbors

    # 2.d Left edge:
    if nodeX ==0:

        pNeighbors = [nodeIdx - idxWidth, nodeIdx +1 , nodeIdx +idxWidth ]
        dNeighbors = [nodeIdx -idxWidth +1 , nodeIdx +idxWidth+1]

        return pNeighbors , dNeighbors


    ### Normal Cases: Node is within the grid:

    pNeighbors = [nodeIdx -1, nodeIdx +1, nodeIdx - idxWidth  ,nodeIdx + idxWidth]

    dNeighbors = [ nodeIdx - idxWidth - 1, nodeIdx - idxWidth + 1 , nodeIdx + idxWidth - 1 , nodeIdx + idxWidth + 1]

    return pNeighbors , dNeighbors



## Description: This function takes as an input three points. Each point is a tuple with x and y co-ordinates:
# It returns either:
# 0: Co-linear points:
# 1: Clockwise orientation
# 2: Counter-clockwise orientation:

## Logic of the function: New approach:

# 1. Consider the ray formed by the first two points.
# 2. Determine whether the thrid point is to the right or left of the ray(This could also correspond to being up or down)
# 3. This can be done by computing the following determinant : SHOW THIS LATER
# 4. If the point is to the right, the triplet is CLOCKWISE!
#    If the point is to the left, the triplet is COUNTER-CLOCKWISE!

# Are there any border cases for this algorithm?

def get_orientation(pa,pb,pc):

    # Compute the determinant:
    det = (pb[0] - pa[0])*(pc[1] - pa[1]) -  (pc[0] - pa[0])*(pb[1] - pa[1])

    # Determine orientation:
    # Case : Point is to the right ->Triplet is clockwise:
    if det > 0:
        return 1

    # Case : Point is to the left ->Triplet is counter-clockwise:
    elif det < 0:

        return 2

    # Case: Points are collinear:
    else:

        return 0


### Description: We are given two segments: s1 and s2:  Each segment is two tuples in a list.Each tuple has two foats.
#                The first two tuple is the co-ordinates of the start point and second is for the end point.

# Example: s1 = [p1,q1]  --> p1=(xstart,ystart) , q1 = (xend,yend)\
#          s2 = [p2,q2]


## Logic used: Taken from Geek for Geeks website:https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

# Segements intersect if one of the following conditions is satisfied:

# Condition # 1: (p1,q1,p2) & (p1,q1,q2) have different orientations and (p2,q2,p1) & (p2,q2,q1) have different orientations

# Condition # 2: All the four triplets are collinear

def checkIntersection(seg1, seg2):

    # Determine the orientations of the different triplets:

    # Triplet #1: (p1, q1, p2)
    p1q1p2 = get_orientation(seg1[0], seg1[1], seg2[0])

    # Triplet #1: (p1, q1, q2)
    p1q1q2 = get_orientation(seg1[0], seg1[1], seg2[1])

    # Triplet #3: (p2, q2, p1)
    p2q2p1 = get_orientation(seg2[0], seg2[1], seg1[0])

    # Triplet #4: (p2, q2, q1)
    p2q2q1 = get_orientation(seg2[0], seg2[1], seg1[1])

    # Check for the general case:Condition #1
    if p1q1p2 != p1q1q2 and p2q2p1 != p2q2q1:

        return True

    # Check for the special case: Condition #2:
    if p1q1p2 == 0 and p1q1q2==0 and p2q2p1 ==0  and p2q2q1 ==0:

        #print ("Speical case reached!")

        return True

    # Otherwise they don't intersect:
    return False



### Description: Assume we are given the node(nodeIdx) at which the robot is. This is the x-y location at which the robot
#                center is. We add a bounding box around the center to account for the width of the robot and we output the
#                corners of the box (as points) and the edges of the box( as segments).

def get_box(nodeIdx):

    #global params.robotWidth, X,Y

    # First get the x-y co-ods of the current node:
    nodeX = params.X[nodeIdx]
    nodeY = params.Y[nodeIdx]


    halfWidth = params.robotWidth/1.999
    
    #print (halfWidth)

    # Now get the x-y co-ordinates of the four corners: As tuples:
    A = (nodeX - halfWidth , nodeY - halfWidth)

    B = (nodeX + halfWidth , nodeY - halfWidth)

    C = (nodeX + halfWidth , nodeY + halfWidth)

    D = (nodeX - halfWidth , nodeY + halfWidth)

    # Now create the four segments:

    seg1 = [A,B]
    seg2 = [B,C]
    seg3 = [C,D]
    seg4 = [D,A]

    # How about store everything in two lists?

    endPoints = [A,B,C,D]

    segments = [seg1, seg2, seg3, seg4]

    return endPoints, segments


### Description: Given a node index(nodeIdx) and a list of the segments in the grid, determine whether the robot can be placed
#                at that node.

# Logic: Create a box of the robot's width at the node. Check whether any of the segments in the list intersect with the
#        box segments.

def is_node_visitable(nodeIdx, mazeSegments):

    # Get the segments of the box around the node:
    endPoints , boxSegments = get_box(nodeIdx)

    # Loop through the maze segments:
    for mazeSeg in mazeSegments:

        # Loop through the box segments:
        for boxSeg in boxSegments:


            # Check if box segment intersects with maze segment:
            if checkIntersection(mazeSeg,boxSeg):
                return False


    # If none of the maze segments intersect with the box segments, then node is visitable:
    return True


### Description: Given the indices of two nodes check whether it is possible to have an edge between those two nodes. Return true if possible

# Logic: Construct a boxes around the two node centers corresponding to the robots width. Join the extreme points to get the
#        extreme segments. Check if the extreme segments intersect with any of the maze segments.


def check_edge(nodeIdx1, nodeIdx2, mazeSegments):

    # Template:
    # if x1 > x2 and y1 > y2: bb, dd
    # if x2 > x1 and y2 > y1: bb,dd
    # if x1> x2 and  y1< y2: aa,cc
    # if x2> x1 and  y2 < y1: aa,cc
    # if x1 == x2 or y1 ==y2: aa, bb

    #global params.robotWidth, X, Y


    # Get the x and y co-ordinates of the nodes:
    x1 = params.X[nodeIdx1]
    y1 = params.Y[nodeIdx1]

    x2 = params.X[nodeIdx2]
    y2 = params.Y[nodeIdx2]

    # Get boxes around the two nodes:
    endPoints1 , segments1 = get_box(nodeIdx1)

    endPoints2 , segments2 = get_box(nodeIdx2)

    A1 = endPoints1[0]
    B1 = endPoints1[1]
    C1 = endPoints1[2]
    D1 = endPoints1[3]

    A2 = endPoints2[0]
    B2 = endPoints2[1]
    C2 = endPoints2[2]
    D2 = endPoints2[3]


    # Check the different conditions:
    if (x1 > x2 and y1 > y2) or (x2 > x1 and y2 > y1):

        seg1 = [A1,A2]
        seg2 = [C1,C2]
        extremeSegs = [seg1, seg2]


    elif (x1>x2 and y1 < y2) or (x2>x1 and y2<y1):

        seg1 = [B1,B2]
        seg2 = [D1,D2]
        extremeSegs = [seg1, seg2]


    elif x1 ==x2 or y1==y2:

        seg1 = [A1,A2]
        seg2 = [B1,B2]
        extremeSegs = [seg1, seg2]


    # Loop through the maze segments:
    for mazeSeg in mazeSegments:

        # Loop through extreme segements:
        for exSeg in extremeSegs:

            if checkIntersection(mazeSeg, exSeg):

                return False


    # If there is no intersection, then return True since an edge is possible:
    return True



### Description: Function to get node index from (x,y) co-ordinates:

def get_node_idx(x,y):

    #global params.widthX , params.gridRes

    idxWidth = int(params.widthX/params.gridRes) + 1

    nodeIdx = int(y/params.gridRes)*idxWidth + int(x/params.gridRes)

    return nodeIdx


### Description: A* algorithm on a grid without pre-computing all the edges on the graph. This saves time and space
#                if A* only needs to be run once and if the shortest path is unlikely to visit all areas of the grid: 
#                Use at your own discretion. 


# Logic: A* differs from Dijkstra only in the sense that an additional heurestic is added to the key. 
# The heurestic used here is the Euclidean distance to the Goal Node: 


# First define the function to compute the heurestic: 
def euclidean_heurestic(currIdx, goalIdx): 
    
    # Extract the X and Y coods of the two nodes: 
    currX = params.X[currIdx] 
    currY = params.Y[currIdx]
    
    goalX = params.X[goalIdx]
    goalY = params.Y[goalIdx]
    
    return  np.sqrt((goalX - currX)**2 + (goalY - currY)**2)
    
def astar_no_graph(startIdx, goalIdx, numNodes, epsilon):
    
    #global mazeSegments
    
    # Parameters: 
    # 1. Edge cost for perpendicular neighbors: 
    pCost = 1
    
    # 2. Edge cost for diagonal neighbors: 
    dCost = 1.414
    
    ## Steps: 
    # 0. Initialize a list for the closed List:
    closed = []
    
    # 0.5: Initialize a boolean array to indicate which nodes have been visited: 
    visited = []    
    for i in range(numNodes):         
        visited.append(False)
    
    
    # 1. Initialize the Min-Heap which will be used as the PQ: 
    PQ =dHeap(numNodes)
    
    # 2. Add the start node to the PQ:
    startNode = dHeapNode(startIdx,0,0) 
    
    PQ.insert(startNode)
    
    # Pop off:
    iterCount = 0
    popped = startNode
    
    # 3. As long as goalIdx is not popped from the PQ and if the PQ is not empty:
    while popped.vertex != goalIdx and (PQ.heapList or iterCount < 1):
        
        # Update the iteration counter:
        iterCount +=1
        
        # Now pop of the next minimum node in the PQ: 
        popped = PQ.extract_min()
        
        # Add the popped node to the closed list: 
        closed.append(popped)
        
        # Mark the node as visited: 
        visited[popped.vertex] = True
        
        # Extract and Iterate through the neighbors of the popped vertex:        
        pNeighbors , dNeighbors = get_neighbor_indices(popped.vertex)
        
        # 1. Perpendicular Neighbors:
        for p in pNeighbors:
            
            # Only 
            # 1. if the neighbor has not been visited previously: 
            # 2. The neighbor is visitable and 
            # 3. An edge between neighbor and popped node is possible:
            if not visited[p] and is_node_visitable(p, params.mazeSegments) and check_edge(popped.vertex,p,params.mazeSegments):
            
                # Get the key (distance from start) of the neighbor node and update the best neighbor in the closed list:
                newKey = popped.key + pCost + epsilon*euclidean_heurestic(p,goalIdx)
                newNode = dHeapNode(p , newKey, iterCount - 1)

                # Insert/update the new node in the Priority Queue:
                PQ.insert(newNode)
        
        # 2. Diagonal Neighbors: 
        for d in dNeighbors: 
            
            # Only 
            # 1. if the neighbor has not been visited previously: 
            # 2. The neighbor is visitable and 
            # 3. An edge between neighbor and popped node is possible:
            if not visited[d] and is_node_visitable(d,params.mazeSegments) and check_edge(popped.vertex,d,params.mazeSegments):
            
                # Get the key (distance from start) of the neighbor node and update the best neighbor in the closed list:
                newKey = popped.key + dCost + epsilon*euclidean_heurestic(p,goalIdx)
                newNode = dHeapNode(d , newKey, iterCount - 1)

                # Insert/update the new node in the Priority Queue:
                PQ.insert(newNode)     

    
    
    # If the PQ is empty then a Path is not feasible: 
    if not PQ.heapList and popped.vertex != goalIdx: 
        
        print (" Path not Possible!!! Sorry!!!")
        return []
        
    # Extract the shortest path by Back tracking:
    pathIndices = []    
    currNode = popped
    
    # Back tracking: 
    while currNode.vertex != startIdx: 
        
        pathIndices.append(currNode.vertex)
        
        currNode = closed[currNode.bestNeighbor]
     
    # Reverse the path: 
    aStarPathIndices = pathIndices[::-1]
    
    # Print the path co-ordinates:
#     for idx in truePathIndices: 
        
#         print( X[idx], ",", Y[idx])
        
    # Return the shortest path: 
    return aStarPathIndices

