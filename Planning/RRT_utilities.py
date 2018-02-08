### Description: RRT planner : Many thanks to Bijo Sebastian for providing this algorithm: 

# Date added : 3rd Feb 2018

import random
import math
import params
import matplotlib.pyplot as plt

#parameters
Window_size = params.windowSize # The operating window for each D.O.F
NUMNODES = params.numNodes # Maximum number of nodes
Epsilon = params.stepSize # For the groeth step

#Define nodes
class Node:
    # The class for node
    x = 0.0
    y = 0.0
    parent = None
    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord

start = Node(0.0, 0.0) # Start
goal = Node(80.0, 5.0) # Goal


def get_orientation(pa,pb,pc):
    #All creidts to Savio
    #To help with the collsion checker

    # Compute the determinant:
    det = (pb[0] - pa[0])*(pc[1] - pa[1]) -  (pc[0] - pa[0])*(pb[1] - pa[1])

    # Determine orientation:
    # Case : Point is to the right -> Triplet is clockwise:
    if det > 0:
        return 1
    # Case : Point is to the left -> Triplet is counter-clockwise:
    elif det < 0:
        return 2

    # Case: Points are collinear:
    else:
        return 0
    
    

def checkIntersect_rrt(n1, n2):
    #All credits to Savio
    # To check if intersections occur

    p1 = [n1.x, n1.y]
    q1 = [n2.x, n2.y]
    
    for mazeSeg in params.mazeSegments:
        
        p2 = mazeSeg[0]
        q2 = mazeSeg[1]

        p1q1p2 = get_orientation(p1, q1, p2)
        p1q1q2 = get_orientation(p1, q1, q2)
        p2q2p1 = get_orientation(p2, q2, p1)
        p2q2q1 = get_orientation(p2, q2, q1)

        # Check
        if p1q1p2 != p1q1q2 and p2q2p1 != p2q2q1:
            return True
        if p1q1p2 == 0 and p1q1q2==0 and p2q2p1 ==0  and p2q2q1 ==0:
            return True

    # Otherwise they don't intersect:
    return False

def dist(n1,n2):
    # Measure euclidean distance between nodes
    return math.sqrt((n1.x - n2.x)*(n1.x - n2.x) + (n1.y - n2.y)*(n1.y - n2.y))

def step_from_to(n1,n2):
    # Grow from current node to destination node as far as you can before collision.

    res = Node(n1.x,n1.y)
    slope = math.atan2((n2.y - n1.y), (n2.x - n1.x))
    temp = Node(n1.x + Epsilon*math.cos(slope), n1.y + Epsilon*math.sin(slope))

    while not checkIntersect_rrt(n1, temp):
        res.x =  temp.x
        res.y =  temp.y
        temp.x += Epsilon*math.cos(slope)
        temp.y += Epsilon*math.sin(slope)

    return res


def drawSolutionPath(sol, nodes, start, goal):
    # To get the solution path out
    nn = nodes[0]
    
    for p in nodes:
        if p != start:

            plt.plot([p.x, p.parent.x], [p.y, p.parent.y], 'b--')

    #Plot sol
    goal.parent = sol
    nn = goal   
    
    path = []
    
    path.append( (nn.x, nn.y))
    
    print (" The RRT path is: ")
    while nn!=start:
        print ("[" ,nn.x, nn.y ,"]" , end= " -->")
        plt.plot(nn.x, nn.y, 'ms')
        plt.plot([nn.x, nn.parent.x], [nn.y, nn.parent.y], 'm--')
        plt.draw()
        plt.grid()
        plt.gca().invert_yaxis()
        nn = nn.parent
        
        path.append((nn.x, nn.y))
        
        

    print (nn.x, nn.y)
    plt.show()
    
    
    return path

def RRT(start, goal):
    # The main function
    nodes = []# The tree of nodes

    nodes.append(start) # Start

    #Setup plot
    plt.close('all')
    plt.axis([0, Window_size, 0, Window_size])
    plt.plot(start.x, start.y, 'go', ms = 10.0)
    plt.plot(goal.x, goal.y, 'go', ms = 10.0)
    
    for mazeSeg in params.mazeSegments:
        plt.plot([mazeSeg[0][0], mazeSeg[1][0]], [mazeSeg[0][1], mazeSeg[1][1]], '-k')

    flag_found  = False
    
    while not flag_found and len(nodes) <= NUMNODES:
        #print ('iter', len(nodes))
        rand = Node(random.random()*Window_size, random.random()*Window_size) #rand is randomly sampled node
        #print(rand.x, rand.y)

        
        flagc = 0
        nn = nodes[0] #nn is nearest node to rand
        
        for p in nodes:

            if dist(p, rand) <= dist(nn, rand) and not checkIntersect_rrt(p, rand):
                nn = p
                flagc = 1
                #print('no coll')

        if flagc == 0:
            #print('inter')
            nn = nodes[0]
            for p in nodes:
                if dist(p, rand) <= dist(nn, rand):
                    nn = p
            rand =  step_from_to(nn,rand)
            #print(rand.x, rand.y)

        if rand.x == nn.x and rand.y == nn.y:
            #print('No motion')
            continue
            
        #Take the random node into the tree
        rand.parent = nn
        #print('parent')
        #print(nn.x, nn.y)
        nodes.append(rand)
        plt.plot(rand.x, rand.y, 'bo', ms = 5)

        if len(nodes)%5 == 0:
            nn = nodes[0]
            
            
            for p in nodes:
                
                if not checkIntersect_rrt(p, goal):
                    
                    nn = p
                    goal.parent = nn
                    
                    path = drawSolutionPath(nn, nodes, start, goal)
                    flag_found  = True
                    return path

    print('failed')
    
    
    

### Vectorized version: 5th Feb 2018: 
def RRT_vec(start, goal):
    # The main function
    nodes = []# The tree of nodes

    nodes.append(start) # Start

    #Setup plot
    plt.close('all')
    plt.axis([0, Window_size, 0, Window_size])
    plt.plot(start.x, start.y, 'go', ms = 10.0)
    plt.plot(goal.x, goal.y, 'go', ms = 10.0)
    
    for mazeSeg in params.mazeSegments:
        plt.plot([mazeSeg[0][0], mazeSeg[1][0]], [mazeSeg[0][1], mazeSeg[1][1]], '-k')

    flag_found  = False
    
    while not flag_found and len(nodes) <= NUMNODES:
        #print ('iter', len(nodes))
        rand = Node(random.random()*Window_size, random.random()*Window_size) #rand is randomly sampled node
        
        q= [rand.x , rand.y]
        
        # Set a flag to check for collisions:
        collision = False
        
        #print(rand.x, rand.y)

        ### ------------------------ VECTORIZING SECTION BEGINS --------------------- ###
        # Extract the x and y co-ods in arrays: px and py are vectors:
        px = np.asarray(p.x for p in nodes)
        py = np.asarray(p.y for p in nodes)
        
        distArr = np.sqrt( (px- rand.x)**2 + (py - rand.y)**2 )
        
        # minDistIdx = np.argmin(distArr)  : LATER!
        
        # Compute the determinant: Only for reference:
        # det = (pb[0] - pa[0])*(pc[1] - pa[1]) -  (pc[0] - pa[0])*(pb[1] - pa[1])
        
        final_collision = np.full((len(nodes),1), False, dtype =bool)
        # For each obstacle segment in the maze:
        for i in range(len(fat_obes)): 
            
            m = [fat_obes[i].xs, fat_obes[i].ys]
            n = [fat_obes[i].xe, fat_obes[i].ye]
            
            # Compute the 4 orientation triplets: This time p is a vector, q (rand) , m and n are single points:
            
            # 1. pqm :
            pqm = np.sign( ( q[0] - px)*(m[1] - py) - (m[0] - px)*(q[1] - py) )
            
            # 2. pqn: 
            pqn = np.sign( ( q[0] - px)*(n[1] - py) - (n[0] - px)*(q[1] - py) )
            
            # 3. mnp: 
            mnp = np.sign( (n[0] - m[0])*(py - m[1]) - (px - m[0])*(n[1] - m[1]) ) 
            
            # 4. mnq: 
            mnq = np.sign( (n[0] - m[0])*(q[1] - m[1]) - (q[0] - m[0])*(n[1] - m[1]) + np.zeros_like(mnp) )
            
            
            # Now check the two cases for intersection: 
            
            # 1. General case: 
            case1 = np.logical_and( pqm != pqn , mnp != mnq)                  
            
            # 2. Special case: 
            case2 = np.logical_and(np.logical_and(pqm ==0 , pqn ==0)  , np.logical_and(mnp == 0 , mnq ==0) )                       
                
            collision = np.logical_or(case1,case2)
            final_collision = np.logical_or(final_collision,collision)
            
        
        if np.all(final_collision):
            nn = nodes[np.argmin(distArr)]
        else:
            distArr[final_collision == True] = np.inf 
            nn  = nodes(np.argmin(distArr))
            
        
        ### ----------------------------------- VECTORIZING SECTION ENDS ------------------------------ ### 
            
        
        flagc = 0
        nn = nodes[0] #nn is nearest node to rand
        
        for p in nodes:

            if dist(p, rand) <= dist(nn, rand) and not checkIntersect_rrt(p, rand):
                nn = p
                flagc = 1
                #print('no coll')

        if flagc == 0:
            #print('inter')
            nn = nodes[0]
            for p in nodes:
                if dist(p, rand) <= dist(nn, rand):
                    nn = p
            rand =  step_from_to(nn,rand)
            #print(rand.x, rand.y)

        if rand.x == nn.x and rand.y == nn.y:
            #print('No motion')
            continue
            
        #Take the random node into the tree
        rand.parent = nn
        #print('parent')
        #print(nn.x, nn.y)
        nodes.append(rand)
        plt.plot(rand.x, rand.y, 'bo', ms = 5)

        if len(nodes)%5 == 0:
            nn = nodes[0]
            for p in nodes:
                if not checkIntersect_rrt(p, goal):
                    nn = p
                    path = drawSolutionPath(nn, nodes, start, goal)
                    flag_found  = True
                    return path

    print('failed')
