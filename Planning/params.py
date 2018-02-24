## VREP Simulation parameters:


## Perception Parameters:



## Planning parameters:

# a. Planner grid parameters:
# Parameters:
# Width of Grid in X-direction : widthX
# Width of Grid in Y-direction : widthY
# Resolution of the grid: gridRes
# Width of the robot (A square for now) - robotWidth
widthX = 60
widthY = 60
gridRes = 2.0
robotWidth = 1.5


# Grid co-ordinates: Initialized later
X = None
Y = None

# Initialized here only during testing phase, else commeneted out: Obtain from VREP or real world: Buck up!
seg1 = [(25,5) ,(45,10)]
seg2 = [(45,68) ,(48,40)]
seg3 = [(46,25), (10,0)]
seg4 = [(26,30), (16,27)]
seg5 = [(14,10),(24,44)]
seg6 = [(40,8),(14,45)]
seg7 = [(10,10),(10,40)]
seg8 = [(32,30),(60,30)]
# seg9 = [(13,20),(13,11)]
# seg10 = [(14,20),(14,11)]
# seg11 = [(15,20),(15,11)]
# seg12 = [(16,20),(16,11)]
# seg13 = [(17,20),(17,11)]



mazeSegments = [seg1, seg2, seg3, seg4 , seg5 , seg6, seg7, seg8]
nSeg = 8



# b. Dijkstra parameters:


# c. A* parameters:
# Weighting factor for heurestic - epsilon
epsilon = 2


# d. RRT parameters: 
windowSize = 50.0 # The operating window for each D.O.F
numNodes = 500 #Maximum number of nodes
stepSize= 0.5 # For the tree growth step
GG = 200

# inflated maze segments:
infMazeSegments = []

# Storage of co-ods of maze segments:
mxVec , myVec , nxVec , nyVec = None , None , None, None

# Storage for x and y co-od sof nodes in the tree: 
pxVec = []
pyVec = []

# Nodes on the Random Tree: 
nodes = []

# e. RRT* parameters: 

radius = 2  # Search radius for the chooseParent function: 

pCost = []  # List which is used to keep track of the cost from the start to each of the nodes



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
