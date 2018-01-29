%Global variables
global vrep; %for VREP
global clientID;

global left_motor_h;%Handles
global right_motor_h;

%Initialise the vrep model
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,true,true,2000,5);

%All in meters (Based on the model you have)
wheel_dia = 0.195;
robot_width = 0.381;

if (clientID>-1)
    disp('Connected to VREP remote API server');
    vrep.simxAddStatusbarMessage(clientID,'Begin Simulation',vrep.simx_opmode_oneshot);    
    
    %Motors
    [~, left_motor_h] = vrep.simxGetObjectHandle(clientID,'left',vrep.simx_opmode_oneshot_wait);
    vrep.simxSetJointForce(clientID, left_motor_h, 10000.0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, left_motor_h, 0.0, vrep.simx_opmode_oneshot);
    
    [~, right_motor_h] = vrep.simxGetObjectHandle(clientID,'right',vrep.simx_opmode_oneshot_wait);
    vrep.simxSetJointForce(clientID, right_motor_h, 10000.0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, right_motor_h, 0.0, vrep.simx_opmode_oneshot);    
    
    %Handles to obain robot state information
    [~, robot_h] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait);
    [~, position] = vrep.simxGetObjectPosition(clientID, robot_h, -1, vrep.simx_opmode_streaming);
    [~, orientation] = vrep.simxGetObjectOrientation(clientID, robot_h, -1, vrep.simx_opmode_streaming);
else
    disp('Failed connecting to remote API server');
    vrep.delete(); % call the destructor!
end


v = 0.05;
w = 0.1;
for  i = 0:100
        
    vel_r = ((2.0*v) + (w*robot_width))/(wheel_dia);
    vel_l = ((2.0*v) - (w*robot_width))/(wheel_dia);
    pause(0.1)    
    vrep.simxSetJointTargetVelocity(clientID, left_motor_h, vel_l, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, right_motor_h, vel_r, vrep.simx_opmode_oneshot);
end

vrep.simxSetJointTargetVelocity(clientID, left_motor_h, 0.0, vrep.simx_opmode_oneshot);
vrep.simxSetJointTargetVelocity(clientID, right_motor_h, 0.0, vrep.simx_opmode_oneshot);
vrep.delete(); 

