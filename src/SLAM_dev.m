%% SLAM ALGORITHM DEVELOPMENT

% RUBEN SANCHEZ BLAZQUEZ

% DESCRIPTION: 
%% MAPA LOAD

close all;
clear all;

NUM_MAP=1; % map selection (1 - 11)

ORIGEN_X = [2.5    2.5  11   14.5   16	  3   10   3   10   10   16];    % rows
ORIGEN_Y = [7.5   10.5	11   13.5   21   10   21  11   22   27   27];    % columns
ORIENT = [0 0 pi/2 pi/2 0 0 pi 0 pi pi 0];

path = strcat(strcat('maps/map',num2str(NUM_MAP)),'.csv');
refMap = csvread(path);

[mapdimx,mapdimy] = size(refMap);
origin=[ORIGEN_X(NUM_MAP), ORIGEN_Y(NUM_MAP)];

%% GENERATION OF OCCUPANCY GRIDS

% grid with the reference map
refMap = binaryOccupancyMap(refMap,1);
refMap = binaryOccupancyMap(refMap,10);
refFigure = figure('Name','Reference Map');
show(refMap);
title('Reference map');

% Empty grid for exploration
emptyMap = binaryOccupancyMap(mapdimy,mapdimx,10);
mapFigure = figure('Name','Explore Map');
show(emptyMap);

%% KINEMATIC MODEL OF THE ROBOT

diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
%% CONTROLLER FOR TRAJECTORY TRACKING

controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);
controller.Waypoints = origin+0.1; % initialization of the required exploration path
%% LIDAR SENSOR MODEL

sensor = rangeSensor;
sensor.Range = [0,5]; % 5 m range

%% INITIAL POSITION

initPose = [origin(1), origin(2), ORIENT(NUM_MAP)];

%% EXPLORATION AND MAP GENERATION

[map,time,dist,surf,robot_pose] = explore(diffDrive,controller,initPose,refMap,emptyMap,refFigure,mapFigure,sensor);
mapGen = figure('Name','Generated Map');
occMatrix = checkOccupancy(map);
show(map)
display(time)
display(dist)
display(surf)
display(robot_pose)

%% NAVIGATION TO THE ORIGIN

navigate(map,controller,diffDrive,robot_pose,origin,refFigure,mapGen);



