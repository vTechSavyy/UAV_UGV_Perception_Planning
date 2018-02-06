### Description: RRT planner : Many thanks to Bijo Sebastian for providing this algorithm:

# Date added : 3rd Feb 2018

import random
import math
import params
import matplotlib.pyplot as plt

#parameters
Window_size = params.Window_size # The operating window for each D.O.F
NUMNODES = params.NUMNODES # Maximum number of nodes
Epsilon = params.Epsilon # For the groeth step

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
    # Case : Point is to the right ->Triplet is clockwise:
    if det > 0:
        return 1
    # Case : Point is to the left ->Triplet is counter-clockwise:
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
                    path = drawSolutionPath(nn, nodes, start, goal)
                    flag_found  = True
                    return path

    print('failed')
