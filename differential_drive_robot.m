% Simulation of a differential drive robot
%Code written By JAIVIN PATEL 22BEE051
clc;
clear;
path=[1 1; 2 2;3 5;4 2;5 7;6 8;7 7;8 10]
initial_location=path(1,:);%first line of path is initial location
goal_location=path(end,:);%end line on path is goal
initial_orientation=0;
pose=[initial_location';initial_orientation];
tend=20;
dt=0.1;
t=0:dt:tend;
robot=differentialDriveKinematics('VehicleInputs','VehicleSpeedHeadingRate');
robot.WheelRadius=0.05;
robot.TrackWidth=1;
frameSize=robot.TrackWidth;
controller=controllerPurePursuit;
controller.Waypoints=path;
controller.DesiredLinearVelocity=0.6;
controller.MaxAngularVelocity=2;
controller.LookaheadDistance=0.3;
goal_radius=0.5;
vizRate=rateControl(1/dt);
distanceTogoal=norm(initial_location-goal_location);
while(distanceTogoal>goal_radius)
    [v,omega]=controller(pose);
    vel=derivative(robot,pose,[v,omega]);
    pose=pose+vel*dt;
    distanceTogoal=norm(pose(1:2)'-goal_location);
    plot(path(:,1),path(:,2),"k--d")
    hold off
    T1=[pose(1);pose(2);0];
    T2=axang2quat([0 0 1 pose(3)])
    plotTransforms(T1',T2,"MeshFilePath","groundvehicle.stl","Parent",gca,"View","2D","FrameSize",frameSize);
    waitfor(vizRate);
end