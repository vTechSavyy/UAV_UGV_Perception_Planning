# UAV_UGV_Perception_Planning
Repository for testing code for UAV_UGV co-operative perception and planning.

Cloning the repo:

git clone https://github.com/vTechSavyy/UAV_UGV_Perception_Planning.git

# Problem Description:
The UGV will be placed in a maze on the ground. The UGV doesn't know the structure of the maze. The UAV hovers above the maze
and is equipped with a downward facing camera. The goal is to use image processing and inference on -board the UAV
to get the structure of the maze and the current location of the UGV. This information is then relayed from UAV -> UGV

With knowledge of the structure of the maze, the UGV is supposed to plan its route out of the maze in the least possible time.
The UGV then implements a controller which takes it out of the maze with the UAV guiding it along the way.

# Perception:
### Objective:
Acquire Images from UAV camera -> Run Canny Edge Detector -> Run Customized Hough Transform -> Get walls of Maze as segments on the Ground Plane

The Customized Hough Transform returns the end points of the segments in the image along with the equations of the lines.

# Planning:

### Comparison  of Planning Algorithms: A*, RRT , RRT with vectorized collision checks and RRT* with vectorized collision checks.

#### 1. Install Jupyter for Python 3:
pip3 install --upgrade pip

pip3 install jupyter

#### 2. Inside the cloned repository run the following command:
jupyter notebook

#### 3. In the Planning folder open the IPython Notebooks:

Open and Run the IPython notebook "Compare_Planning_Algorithms"

Use shift-Enter to run each cell and move to next. 


# Videos:

## Weighted A* algorithm - Pioneer P3DX:

https://www.youtube.com/watch?v=xAdVCyKFT00

## RRT algorithm - Pioneer P3DX

...Coming soon ...

## RRT* algorithm - Pioneer P3DX

...Coming soon ...
