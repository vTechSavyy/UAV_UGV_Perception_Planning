# Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved.
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
#
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
#
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import numpy as np
import params
from   utilities import *
from   RRT_vect_utilities import *

global angErrHist
global distErrHist

angErrHist = []
distErrHist = []

print ('Program started')

# ---------------- DEFINE CUSTOM FUNCTIONS HERE --------------#
# 1. Function to get the end points of a single wall segment:
def get_wall_seg(centerPos, length, zAngle):

    # TO DO: Put detailed formula in comments after testing works successfully!!!
    cosA = np.cos(zAngle)
    sinA = np.sin(zAngle)

    # Start point:
    xs = centerPos[0]  - sinA*(length/2)
    ys = centerPos[1]  + cosA*(length/2)

    # End point:
    xe = centerPos[0]  + sinA*(length/2)
    ye = centerPos[1]  - cosA*(length/2)

    return [(xs,ys) , (xe,ye)]

# 2. Function to get the maze sgements from the wall handles list:
def get_maze_segments(wallHandles):

    ## Get handles to:
    # 1. The Maze collection:
    res , mazeHandle = vrep.simxGetCollectionHandle(clientID, "Maze", vrep.simx_opmode_blocking)

    # Get the handles associated with each wall and the absolute position of the center of each wall:
    res, wallHandles , intData, absPositions, stringData = vrep.simxGetObjectGroupData(clientID, mazeHandle, 3,vrep.simx_opmode_blocking)

    mazeSegments = []

    if res == vrep.simx_return_ok:

        count = 1
        for wall in wallHandles:

            res, wallCenterAbsPos = vrep.simxGetObjectPosition(clientID, wall, -1, vrep.simx_opmode_oneshot_wait)

            res, wallMinY = vrep.simxGetObjectFloatParameter(clientID,wall, vrep.sim_objfloatparam_objbbox_min_y , vrep.simx_opmode_oneshot_wait)

            res, wallMaxY = vrep.simxGetObjectFloatParameter(clientID,wall, vrep.sim_objfloatparam_objbbox_max_y , vrep.simx_opmode_oneshot_wait)

            wallLength = abs(wallMaxY - wallMinY)

            # Get the orientation of the wall: Third euler angle is the angle around z-axis:
            res , wallOrient = vrep.simxGetObjectOrientation(clientID, wall, -1 , vrep.simx_opmode_oneshot_wait)

            # Get the end points of the maze wall: A list containing two tuples: [ (xs,ys) , (xe,ye)]
            wallSeg = get_wall_seg(wallCenterAbsPos, wallLength, wallOrient[2])    # Assuming all walls are on the ground and nearly flat:

            print ("Wall #" , count, " -> " , wallSeg)

            mazeSegments.append(wallSeg)

            count+=1

    else:

        print (" Failed to get individual wall handles!")


    return mazeSegments


# 3. Routine to get the Distance and angular errors for the go-to-goal PID controller:
def g2g_xy_errors(clientID, pioneerHandle, goal):

    # a: Stream current position and orientation:
    res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)
    res , pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)

    xp = pioneerPosition[0]
    yp = pioneerPosition[1]
    thetap = pioneerOrientation[2]

    # # VREP orientation returns a value between -pi to +pi. Trnasform this to the range o to 2*pi
    # if thetap < 0:
    #
    #     thetap = 2*np.pi + thetap
    #
    # print ("Thetap -> " , thetap)

    xg = goal[0]
    yg = goal[1]

    # b. Compute orientation error:
    targetOrientation = np.arctan2( yg - yp , xg - xp)

    # # Arctan2 returns a value between -pi to +pi. Trnasform this to the range o to 2*pi
    # if targetOrientation < 0:
    #
    #     targetOrientation = 2*np.pi + targetOrientation
    #
    # print ("Target -> " , thetap)

    errAng = targetOrientation - thetap


    # Restrict to -pi and +pi:
    errAng = np.mod((errAng+ np.pi), 2*np.pi) - np.pi


    if abs(errAng) < params.angErrThresh:

        reachedOrientation = True
    else:
        reachedOrientation = False

    # c. Compute distance error:
    errDist = np.sqrt( (xg - xp)**2 + (yg - yp)**2)

    if errDist < params.distErrThresh:

        reachedGoal = True
    else:

        reachedGoal = False


    # Return the values:
    return errAng, errDist, reachedOrientation , reachedGoal


# 4. Function to map unicycle commands to differential drive commands:
def diff_drive_map(v,omega, trackWidth,wheelRadius):

    # TO DO: Document the formulas in detail here:

    # 1. Left wheel angular velocity:
    omegaLeft = (v + 0.5*trackWidth*omega)/wheelRadius

    # 2. Right wheel angular velocity:
    omegaRight = (v - 0.5*trackWidth*omega)/wheelRadius


    return omegaLeft , omegaRight


# 5. Go to Goal PID control from a 2-D differential drive robot:
def g2g_controller_PID(clientID, pioneerHandle, pioneerLeftMotorHandle, pioneerRightMotorHandle, goal):

    global angErrHist , distErrHist

    # a. Setup internal parameters of the PID controller:
    reachedGoal = False
    reachedOrientation = False
    wheelRadius = 0.0975   # meters - From VREP model of left wheel
    trackWidth = 0.4       # meters - This is approximate
    v= 0
    omega = 0

    timer = 0

    # Continue this as long as the Robot does not reach its goal:
    while not reachedGoal:

        errAng, errDist, reachedOrientation , reachedGoal = g2g_xy_errors(clientID, pioneerHandle, goal)

        # print (" Time = " , timer , " sec" , end= " -> ")
        # print ("Angular Error =  " , errAng, end= " -> ")
        # print (" Distance Error = " , errDist)

        # Append error values to the cache variabes:
        angErrHist.append(errAng)
        distErrHist.append(errDist)

        # Orientation correction:
        if not reachedOrientation:

            # PID control law for Orientation:
            v = 0

            if len(angErrHist) > params.integralWindow:
                omega = -params.kpTheta*errAng - params.kdTheta*(errAng - angErrHist[-2]) - params.kiTheta*sum(angErrHist[-params.integralWindow:])

            elif len(angErrHist) > 2:

                omega = -params.kpTheta*errAng - params.kdTheta*(errAng - angErrHist[-2]) - params.kiTheta*sum(angErrHist)

            else:

                omega =  -params.kpTheta*errAng


            omegaLeft , omegaRight  = diff_drive_map(v,omega,trackWidth,wheelRadius)

            res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
            res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)

        # Distance correction:
        else:

            # Clear the angular error history cache:
            angErrHist = []

            omega = 0.0

            if  len(distErrHist) > params.integralWindow:
                v = params.kpDist*errDist + params.kdDist*(errDist - distErrHist[-2]) + params.kiDist*sum(distErrHist[-params.integralWindow:])

            elif len(distErrHist) > 2:

                v = params.kpDist*errDist + params.kdDist*(errDist - distErrHist[-2]) + params.kiDist*sum(distErrHist)

            else:
                v = params.kpDist*errDist


            omegaLeft , omegaRight  = diff_drive_map(v,omega,trackWidth, wheelRadius)

            res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
            res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)


        # Set the frequency of the controller:
        time.sleep(0.5)


    # Notify the user:
    #print(" Robot reached its Goal!")

    # Clear the error history caches:
    angErrHist = []
    distErrHist = []

    v= v/2
    omega = 0

    omegaLeft , omegaRight  = diff_drive_map(v,omega,trackWidth,wheelRadius)

    # Deactivate joint actuations:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)


    return reachedGoal




# ---------------------------------- MAIN SCRIPT BEGINS HERE -----------------------------------------------------------------------------#

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:

    print ('Connected to remote API server')

    ### Step 1: Extract the maze segments from the VREP simulation:
    # Format: Returns a list called mazeSegments:
    # mazeSegments is a list of segments -> Each segment is a list of two tuples corresponding to start and end points -> Each tuple contains (x,y) co-ods  of the points

    params.mazeSegments = get_maze_segments(clientID)

    ### Step 2: Get a handles to Pioneer and Quadcopter:
    # 2.a: Handle the Pioneer:
    res , pioneerHandle = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_blocking)
    res,  pioneerLeftMotorHandle = vrep.simxGetObjectHandle(clientID, "left", vrep.simx_opmode_blocking)
    res,  pioneerRightMotorHandle = vrep.simxGetObjectHandle(clientID, "right", vrep.simx_opmode_blocking)

    # Get the position of the Pioneer for the first time in streaming mode: Still have to understand why this is done!
    res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_streaming)
    res , pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1 , vrep.simx_opmode_streaming)

    # Deactivate joint actuations:Make sure Pioneer is stationary:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0, vrep.simx_opmode_streaming)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0, vrep.simx_opmode_streaming)


    # 2.b Handle the Quadcopter:
    # res , quadCameraHandle = vrep.simxGetObjectHandle(clientID, 'Quadricopter_floorCamera', vrep.simx_opmode_blocking)
    # print(res)
    # time.sleep(5)
    #res , quadHandle = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_blocking)

    # if res == vrep.simx_return_ok:
    #
    #     print (" Got the Quadcopter handle")

    ### Step 3: Start the Simulation: Keep printing out status messages!!!
    res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    if res == vrep.simx_return_ok:

        print ("---!!! Started Simulation !!! ---")



    ### Step 3: Perception:  TO BE DONE:

    # # First call:
    # res,resolution,image=vrep.simxGetVisionSensorImage(clientID, quadCameraHandle,0,vrep.simx_opmode_streaming)
    # print ('firts call',res)
    # time.sleep(5)
    # res,resolution,image=vrep.simxGetVisionSensorImage(clientID, quadCameraHandle,0,vrep.simx_opmode_buffer)
    # print ('second call',res)
    #
    # if res == vrep.simx_return_ok:
    #
    #     print (" successful first image!")
    #
    # time.sleep(5)
    #
    # while (vrep.simxGetConnectionId(clientID) != -1):
    #
    #      res, resolution, image = vrep.simxGetVisionSensorImage(clientID, quadCameraHandle, 0, vrep.simx_opmode_buffer)
    #
    #      print (res)
    #
    #      if res == vrep.simx_return_ok:
    #          print (len(image))

    ### Step 4: Planning: Find the shortest path from current location to Goal:

    # Generate grid and return X and Y co-ordinates of all the nodes:
    [params.X,params.Y] = generate_grid()

    ### 1.  A* with out graph for the planning:
    #
    # start = time.time()
    # numNodes = len(params.X)
    # startIdx = get_node_idx(0,0)
    # goalIdx =  get_node_idx(7.0,7.0)
    #
    # pathPointIndices= astar_no_graph(startIdx,goalIdx, numNodes, params.epsilon)
    #
    #
    # end = time.time()
    #
    # timeNoGraph = end - start
    #
    # print (" Time taken for A* without the Graph is : " , timeNoGraph ,  "secs")
    #
    # print (" The generate path is: ")
    # for idx in pathPointIndices:
    #
    #     print( idx , end = "-->")
    #
    # print (" ")



    ### 2. RRT for the planning: Vectorized version: Gracias Senore Bijo!
    # ----------------------------------------------------- #
    # 1. Give the inputs to the RRT algorithm:
    start = Node(0.0, 0.0) # Start
    goal = Node(7.0, 7.0) # Goal

    # 2. Inflate the maze segments:
    inflate_segments()

    # 3. Setup for vectorization:
    setup_vec()

    # 4. Run the RRT algorithm and directly plot the path:
    startTime = time.time()
    RRT_path = RRT(start, goal)
    endTime = time.time()


    # 5. Get the true path from start to goal:
    revPath = reverse_path(RRT_path)

    timeRRT= endTime - startTime

    print (" Time taken for RRT Planner is : " , timeRRT ,  "secs")
    print (" The RRT path is : ")
    print (revPath)
    # ----------------------------------------------------- #


    # Start a timer:
    sTime = time.time()

    pathPointCount = 0

    # Loop to traverse RRT Path:
    for immGoal in revPath:

        # Run the Go-to-goal control function:
        reached = g2g_controller_PID(clientID, pioneerHandle, pioneerLeftMotorHandle, pioneerRightMotorHandle, immGoal)

        if reached:

            # Increment the path point count:
            pathPointCount +=1

            print (" Reached node # " , pathPointCount )
            print (" ---------------------------------")

        else:

            print(" There was an ERROR! Please investigate!!!")



    # # Loop to traverse the planned path by A*:
    # while pathPointCount < len(pathPointIndices)-1 :
    #
    #     immGoalIdx = pathPointIndices[pathPointCount]
    #
    #     # Get the x-y co-ordinates of the immediate goal:
    #     immGoal = (params.X[immGoalIdx] , params.Y[immGoalIdx])
    #
    #     # Run the Go-to-goal control function:
    #     reached = g2g_controller_PID(clientID, pioneerHandle, pioneerLeftMotorHandle, pioneerRightMotorHandle, immGoal)
    #
    #     if reached:
    #
    #         # Increment the path point count:
    #         pathPointCount +=1
    #
    #         print (" Reached node # " , pathPointCount )
    #         print (" ---------------------------------")
    #
    #     else:
    #
    #         print(" There was an ERROR! Please investigate!!!")


    eTime = time.time()

    travelTime = eTime - sTime

    # Display successful path planning message!:
    print ("--- Robot has Successfully reached its final Goal --- ")
    print (" The toal time taken is : " , travelTime , " secs")

    # Set the joint velocities back to zero:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0, vrep.simx_opmode_streaming)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0, vrep.simx_opmode_streaming)


    # Stop the simulation:
    res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    if res == vrep.simx_return_ok:

        print (" --- !!!Stopped Simulation!!! ---")


    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

else:

    print ('Failed connecting to remote API server')



print (' --- Python Remote API Program ended ---')




# ---------------------- TESTING STUFF -----------------------#

    # # Test this out:
    # count =0
    # while count <10:
    #
    #     res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)
    #
    #     print ("Position is: " , pioneerPosition)
    #
    #     # Set the motor velocities:
    #     res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 2, vrep.simx_opmode_streaming)
    #     res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 1.5, vrep.simx_opmode_streaming)
    #
    #     count +=1
    #
    #     time.sleep(1)
