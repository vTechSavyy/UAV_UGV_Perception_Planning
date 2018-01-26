# UAV_UGV_Perception_Planning
Repository for testing code for UAV_UGV co-operative perception and planning

# Problem Description: 
The UGV will be placed in a maze on the ground. The UGV doesn't know the structure of the maze. The UAV hovers above the maze 
and is equipped with a downward facing camera. The goal is to use image processing and inference on -board the UAV
to get the structure of the maze and the current location of the UGV. This information is then relayed from UAV -> UGV

With knowledge of the structure of the maze, the UGV is supposed to plan its route out of the maze in the least possible time. 
The UGV then implements a controller which takes it out of the maze with the UAV guiding it along the way. 
