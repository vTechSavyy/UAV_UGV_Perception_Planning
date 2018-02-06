## VREP Simulation parameters:


## Perception Parameters:



## Planning parameters:

# a. Planner grid parameters:
# Parameters:
# Width of Grid in X-direction : widthX
# Width of Grid in Y-direction : widthY
# Resolution of the grid: gridRes
# Width of the robot (A square for now) - robotWidth
widthX = 10
widthY = 10
gridRes = 0.5
robotWidth = 0.75


# Grid co-ordinates: Initialized later
X = None
Y = None

# Initialized here only during testing phase, else commeneted out: Obtain from VREP or real world: Buck up!
seg1 = [(2,0) ,(2,7)]
seg2 = [(4,6) ,(4,10)]
seg3 = [(6,2), (9,2)]
seg4 = [(6,0), (6,7)]
# seg5 = [(4,0),(4,4)]
# seg6 = [(4,0),(4,4)]
# seg7 = [(11,20),(11,11)]
# seg8 = [(12,20),(12,11)]
# seg9 = [(13,20),(13,11)]
# seg10 = [(14,20),(14,11)]
# seg11 = [(15,20),(15,11)]
# seg12 = [(16,20),(16,11)]
# seg13 = [(17,20),(17,11)]



mazeSegments = [seg1, seg2, seg3, seg4 , seg5 , seg6, seg7, seg8, seg9, seg10,seg11,seg12,seg13]

Sx =[]
Ex =[]
Sy =[]
Ey =[]


# b. Dijkstra parameters:


# c. A* parameters:
# Weighting factor for heurestic - epsilon
epsilon = 2


# d. RRT parameters: 
Window_size = 20.0 # The operating window for each D.O.F
NUMNODES = 100 # Maximum number of nodes
Epsilon = 1 # For the groeth step


## Controller parameters:
# Proportional gain for go-to-goal controller for orientation - kpTheta
# Proportional gain for go-to-goal controller for distance - kpDist
# Derivative gain for go-to-goal controller for orientation - kdTheta
# Derivative gain for go-to-goal controller for distance - kdDist
# Frequency of controller - freqCont - (In Hz)
# Threshold on the angular error - angErrThresh - radians
# Threshold on the distance error - distErrThresh - Parameters

kpTheta = 0.04
kpDist = 0.2

angErrThresh = 0.01
distErrThresh = 0.1
