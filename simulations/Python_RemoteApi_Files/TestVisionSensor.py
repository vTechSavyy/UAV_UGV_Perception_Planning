#!/usr/bin/env python


### Author: Savio Pereira

### Date Created: 10th Feb 2018

### Last Modified: 11th Feb 2018

### Description: This file provides an implementation for the Percpetion pipeline in the UAV_UGV_Planning_Perception project:

### Objective:  The code in this script does the following:

# 1. Acquire images from the VREP simulation scene - Uses the first images in the first 5 seconds to gather information about the maze:

# 2. Runs the customized Hough Transform on the images to get the mazeSegments in Global co-ordinates. This needs information about the position and orientation of the UAV as well as
#    the camera parameters.

# 3. Run a suitable tracker on the image stream so that Pioneer motion can be tracked:
#    The region of interest will first need to be selected manually or done through template/feature matching.


### Import some cool libraries:
import vrep
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import cv2
from collections import deque

## Setup global variables:
refPt = []
count =0
done = False
gotROI = False

# initialize the
# list of tracked points
pts = deque(maxlen=20)
pts2 = deque(maxlen=20)


### Function to get Region of interest in the Image stream :
##  Define the mouse callback function here:
# Arguments to this function are:
# 1. event - Which I think is the type of click that occured
# 2. x , y - X and Y pixel locations of the point selected.
# 3. flags
# 4. params
def click_and_crop(event,x ,y , flags, params):

    # Make the reference points global variables:
	global refPt, count, done

    # Create a if -else case to analyze the type of mouse click:
	if event == cv2.EVENT_LBUTTONDOWN:

		if not done:
			refPt.append((x,y))
			count = count + 1
			print (" Point selected is : ", x, ",", y)

	if event == cv2.EVENT_RBUTTONDOWN:

		done = True

print (' --- Connecting to VREP... ---')

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID==-1:
	print ('Unable to connect to VREP')
	vrep.simxFinish(-1)
	sys.exit(0)

print ('--- Connected to VREP ---')

vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)


# show the frame to our screen
cv2.namedWindow("UAV_downward_camera_feed", cv2.WINDOW_FULLSCREEN)
#cv2.setWindowProperty("UAV_downward_camera_feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# Set the mouse callback:
cv2.setMouseCallback("UAV_downward_camera_feed", click_and_crop)


# Get OpenCV version being used:
version = cv2.__version__
major_ver, minor_ver, subminor_ver = version.split('.')

# Check version: For OpenCV 3.3 and later use the other command:
if int(minor_ver) < 3:
	trackerMIL = cv2.Tracker_create('MIL')
else:
	trackerMIL = cv2.TrackerMIL_create()


print(cv2.__version__)


# 2. In a loop acquire the images at a frequency of 10 Hz:

while True:

	res,v0=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait)

	err, resolution, image_list = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_oneshot_wait)

	if err==vrep.simx_return_novalue_flag:
		print (" Unable to Acquire Image from Vision Sensor")

	image = np.array(image_list,dtype=np.uint8)
	image.resize([resolution[1],resolution[0],3])

	if count < 2 and not gotROI:

		cv2.putText(image, " PLEASE SELECT THE DESIRED REGION OF INTEREST", (200, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255))

    # Check whether the user has selected more than two points by left clicking and then draw a rectangle
    # between those points:
	if count > 1 and not gotROI:

		for i in range(0, count - 1):

			# Draw a rectangle instead:
			cv2.rectangle(image, refPt[i], refPt[i + 1], (0, 0, 255), 2)

            # Draw a line between the previous point and current point:
            # cv2.line(frame, refPt[i], refPt[i+1], (0, 255, 0), 2)

    # If the user has finished selecting the bounding region then apply the detector now:
	if done and not gotROI:

		# # Compute the start row and column for the region of interest:
		(r, c) = (refPt[0][0], refPt[0][1])
		(endr, endc) = (refPt[1][0], refPt[1][1])


        # Compute the width and height of the window:

        # Setup the bounding box:
		bbox = (r,c, endr -r, endc -c)  # Bounding box is specified as : ( x, y, w , h):


        # Initialize the MIL tracker:
		okMIL = trackerMIL.init(image, bbox)


        # Set the flag to true:
		gotROI = True



    # If ROI has been selected:
	if gotROI:

        # Update the MIL tracker:
		okMIL, newboxMIL = trackerMIL.update(image)

        # Update the KCF tracker:
        #okKCF, newboxKCF = trackerKCF.update(image)

        # IF MIL tracker is working then draw a box and put the text in it:
		if okMIL:
			p1 = (int(newboxMIL[0]), int(newboxMIL[1]))
			p2 = (int(newboxMIL[0] + newboxMIL[2]), int(newboxMIL[1] + newboxMIL[3]))
			cv2.rectangle(image, p1, p2, (255,0,0))
			cv2.putText(image, " MIL Tracker", (10 , 10 ), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0))

			cx = int(newboxMIL[0] + newboxMIL[2]/2)
			cy = int(newboxMIL[1] + newboxMIL[3]/2)

			center =(cx,cy)

            # update the points queue
			pts.appendleft(center)

            ## Section 5: Plot the trajectory of the Centers of the detected object:

            # loop over the set of tracked points
			for i in range(1, len(pts)):

				# if either of the tracked points are None, ignore
                # them
				if pts[i - 1] is None or pts[i] is None:

					continue

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
				thickness = int(np.sqrt(20/ float(i + 1)) * 1.5)
				cv2.line(image, pts[i - 1], pts[i], (255, 0, 0), thickness)



	cv2.imshow('UAV_downward_camera_feed',image)

	time.sleep(0.1)

	if cv2.waitKey(1) & 0xFF == ord('q'):

		break


print (" --- Stopping Simulation --- ")

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
vrep.simxFinish(clientID)
