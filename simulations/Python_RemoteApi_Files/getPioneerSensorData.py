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
#from matplotlib import pyplot as plt

# Setup parameters:
sampleRate = 0.5
sampleLimit = 50


# ---------------------------------- DEFINE UTILITY FUNCTIONS HERE -----------------------------------------------------------------------#

# 1. Get accelerometer Data:
def get_accel_data(clientID, accelHandle, accelMassHandle):

    # Retreive accelerometer signals:
    res,state,force, torque = vrep.simxReadForceSensor(clientID, accelHandle, vrep.simx_opmode_buffer)

    accel = []

    if res == vrep.simx_return_ok:

        accel= [force[0]/mass, force[1]/mass, force[2]/mass]


    return res, accel


# 2. Get Gyroscope data:


# ---------------------------------- MAIN SCRIPT BEGINS HERE -----------------------------------------------------------------------------#


### Step 1: Connect to the VREP server:
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:

    print ('Connected to remote API server')


### Step 2: Obtain handles to the different sensors:

# a. Accelerometers:
res1 , accelHandle = vrep.simxGetObjectHandle(clientID, "Accelerometer_forceSensor", vrep.simx_opmode_blocking)
res2,  accelMassHandle = vrep.simxGetObjectHandle(clientID,"Accelerometer_mass", vrep.simx_opmode_blocking)

# Get the mass of the accelerometer parent:
res3, mass = vrep.simxGetObjectFloatParameter(clientID, accelMassHandle, vrep.sim_shapefloatparam_mass,vrep.simx_opmode_oneshot_wait)


# Stream the data the first time:
res, state,force, torque = vrep.simxReadForceSensor(clientID, accelHandle, vrep.simx_opmode_streaming)

if res != vrep.simx_return_ok:
    print (" --- Error getting accelerometer data for the first time ( Can be ignored!)--- ")

if res1 != vrep.simx_return_ok and res2 != vrep.simx_return_ok and res3 != vrep.simx_return_ok:

    print (" --- There was an error getting the Accelerometer Handle ---")


# b. Gyroscopes:
res , gyroHandle = vrep.simxGetObjectHandle(clientID, "GyroSensor", vrep.simx_opmode_blocking)

if res != vrep.simx_return_ok:

    print (" --- There was an error getting the Gyroscope Handle ---")


res1 , gyroX = vrep.simxGetFloatSignal(clientID, 'gyroX', vrep.simx_opmode_streaming)
res2 , gyroY = vrep.simxGetFloatSignal(clientID, 'gyroY', vrep.simx_opmode_streaming)
res3 , gyroZ = vrep.simxGetFloatSignal(clientID, 'gyroZ', vrep.simx_opmode_streaming)

if res1 != vrep.simx_return_ok and res2 != vrep.simx_return_ok and res3 != vrep.simx_return_ok:
    print (" --- Error getting GyroSensor data for the first time ( Can be ignored!)--- ")


# c. GPS:
res , gpslHandle = vrep.simxGetObjectHandle(clientID, "GPS", vrep.simx_opmode_blocking)

if res != vrep.simx_return_ok:

    print (" --- There was an error getting the GPS Handle ---")


### Step 3: Start the simulation:

res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

if res == vrep.simx_return_ok:

    print (" !!! --- Started Simulation --- !!!")



# fig , axes = plt.subplots(3,1, sharex = True)
# plt.ion()

### Step 4: Acquire and plot the data from the handles:
sampleCount = 1


while sampleCount < sampleLimit:

    # a. Accelerometer data: Sampled at a uniform rate of sampleRate per sec:
    res , accelData = get_accel_data(clientID, accelHandle, mass)

    if res != vrep.simx_return_ok:

        print (" --- There was an error getting the accelerometer data --- ")

    #print (accelData)




    # b. Gyroscope data: Sampled at a uniform rate of sampleRate per sec: Using Float Signals here:

    res1 , gyroX = vrep.simxGetFloatSignal(clientID, 'gyroX', vrep.simx_opmode_buffer)
    res2 , gyroY = vrep.simxGetFloatSignal(clientID, 'gyroY', vrep.simx_opmode_buffer)
    res3 , gyroZ = vrep.simxGetFloatSignal(clientID, 'gyroZ', vrep.simx_opmode_buffer)

    if res1 != vrep.simx_return_ok and res2 != vrep.simx_return_ok and res3 != vrep.simx_return_ok:

        print (" --- There was an error getting the Gyro Sensor  data --- ")

    # c. GPS data: Sampled at a uniform rate of sampleRate per sec:
    # -- TO DO : Have to understand how communication tubes work! --#


    # axes[0].plot(sampleCount , gyroX, 'g*')
    # axes[1].plot(sampleCount , gyroY, 'm*')
    # axes[2].plot(sampleCount , gyroZ, 'b*')
    # plt.pause(0.1)


    # d. Get the velodyne data:
    velodyne_points= vrep.simxCallScriptFunction(clientID, 'velodyneHDL_64E_S2', 1, 'VF',[],[],[],[],vrep.simx_opmode_blocking)



    # Increment the counter:
    sampleCount +=1

    # Set the frequency of sampling:
    time.sleep(sampleRate)

plt.show()
