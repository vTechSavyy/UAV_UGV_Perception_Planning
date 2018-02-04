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

mazeSegments = [seg1, seg2, seg3, seg4]

# b. Dijkstra parameters:


# c. A* parameters:
# Weighting factor for heurestic - epsilon
epsilon = 2


# d. RRT parameters: 
Window_size = 10.0 # The operating window for each D.O.F
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
