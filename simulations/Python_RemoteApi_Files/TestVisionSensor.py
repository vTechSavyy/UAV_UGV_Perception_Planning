#!/usr/bin/env python

import vrep
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import cv2

print ('Connecting to VREP...')

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID==-1:
	print ('Unable to connect to VREP')
	vrep.simxFinish(-1)
	sys.exit(0)

print ('Connected to VREP')

vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)




#camera image
res,v0=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait)
time.sleep(1)
err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_oneshot_wait)



if err==vrep.simx_return_novalue_flag:
	'this isnt working'

print (" The length of values in image list is: ")
print (len(image))


print (" The values in the image are like: ")
print (image[1:10])

while True:
    img = np.array(image,dtype=np.uint8)
    img.resize([resolution[1],resolution[0],3])
    cv2.imshow('image',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
vrep.simxFinish(clientID)
