function [exploreMap,tiempo,dist_rec,sup_explo,last_pose] = explore(diffDrive,ppControl,initPose,refmap,exploreMap,fig1,fig2,lidar)

tic % time count init
num_free_cells = 0; % n. of empty cells
num_sensed_cells = 0; % n. of sensed cells
index = 0; % index to fill the vector of sensed cell positions
iindex = 0; % index to fill the vector of obstacle positions
dist_rec = 0; % traveled distance
waypoints_idx = 2; % waypoints index (starts on 2 because of init_pose)

refOccMatrix = checkOccupancy(refmap); % real map occupancy data
[mapdimx,mapdimy] = size(refOccMatrix);

for c = 1:mapdimx 
    for r = 1:mapdimy 
        if (refOccMatrix(c, r) == 0)
            num_free_cells = num_free_cells + 1;
        end
    end
end

coord_sensed_cells = zeros(num_free_cells, 2); % coords of explored cells
coord_obstacle_cells = zeros(0.1*length(find(refOccMatrix == 1)), 2);

% Empty grid that will accumulate the explored cells
sensedMap = occupancyMap(mapdimy,mapdimx); % It is not binary in order to compare with the unsensed cells (value -1)
sensedFigure = figure('Name','Explored Surface');
%show(sensedMap);

sampleTime = 0.05;            % Sample time [s]
t = 0:sampleTime:100;         % Time array
poses = zeros(3,numel(t));    % Pose matrix
poses(:,1) = initPose';

% set rate to iterate at
r = rateControl(1/sampleTime);

% Get the axes from the figures
ax1 = fig1.CurrentAxes;
ax2 = fig2.CurrentAxes;

for idx = 1:numel(t)
    
    % NEW ITERATION POSE
    currPose = poses(:,idx)';
    currPos = currPose(1:2);
    
    
    % MEASUREMENT
    [ranges, angles] = lidar(currPose, refmap);
    
    
    % OBTAINING THE POSITIONS OF THE LASER IMPACT POINTS 
    max_range = lidar.Range(2);
    % The intersection points are obtained (points that are out of range are returned as NaN)
    positions = rayIntersection(refmap,currPose,angles,max_range);
    % To obtain the coordinates of the points where the laser does not collide (out of range), polar coordinates are used
    % Additionally, for points that touch obstacles, their position is saved on the map and in a vector
    for inde = 1:length(positions)
        if (isnan(positions(inde,:)))
        	[x,y] = pol2cart((currPose(3) + angles(inde)),max_range);
        	positions(inde,1) = x + currPos(1); % displacement in x from the origin is added
        	positions(inde,2) = y + currPos(2); % displacement in y from the origin is added
        else
            setOccupancy(exploreMap,positions(inde,:),1,'local'); % are added as occupied space to the sensing map
            for w = 1:length(coord_obstacle_cells) % Each detected obstacle is compared with all stored obstacles so as not to overwrite them
                agregar = true;
                grid_pos = world2grid(exploreMap,positions(inde,:));
                if (grid_pos(1) == coord_obstacle_cells(w,1) && grid_pos(2) == coord_obstacle_cells(w,2)) % break if is already stored
                    agregar = false;
                    break;    
                end
            end
            if (agregar == true)
                iindex = iindex + 1;
                coord_obstacle_cells(iindex,:) = grid_pos; % They are stored in a grid for comparison (due to the rounding problem)
            end
        end    
    end
    
    figure(2)
    show(exploreMap);
    title('Exploration path');
    hold on
    plot(ppControl.Waypoints(:,1),ppControl.Waypoints(:,2), 'o-');
    hold off
    
    
    % Run the Pure Pursuit controller and convert output to wheel speeds
    % THE CONTROLLER DETERMINES THE SPEEDS (LINEAR AND ANGULAR) FOR
    % REACH THE NEW POSITION
    [vRef,wRef] = ppControl(poses(:,idx));
    

    % Perform forward discrete integration step
    % WITH THOSE SPEEDS THE ROBOT MODEL DETERMINES THE SPEEDS
    % LINEAR AND ANGULAR VELOCITY THAT ALLOW THE PREVIOUS MOVEMENT
    vel = derivative(diffDrive, poses(:,idx), [vRef wRef]);
    
    
    % INTEGRATING IN TIME THE NEW POSITION IS OBTAINED
    poses(:,idx+1) = poses(:,idx) + vel*sampleTime; % new pose = actual pose + integral in sampletime
    
    
    % AUTONOMOUS NAVIGATION ALGORITHM (OBTAINING NEW WAYPOINTS)
    resto = mod(idx,24); % Each iteration a new waypoint is calculated
    if (resto == 0 || idx == 1) 
        poss_poses_A = [world2grid(exploreMap,positions), angles]; % first filter based on the maximum sensed distance (grid positions)
        poss_poses_B = zeros(1,3); % second filter based on unexplored coordinates
        i = 0; 
        indxx = find(isnan(ranges)); % positions corresponding to the maximum range detected by the laser
        
        % There are ranges corresponding to the lidar's max_range (NaN)
        poss_poses_A = poss_poses_A(indxx,:); % coordinates and angles corresponding to the points of maximum range
        for k = 1:length(poss_poses_A) % each possible position is compared
            % If there is a viable path, continue, if not, try the next position
            xy = grid2world(exploreMap,poss_poses_A(k,1:2));
            viable = viablepath(ppControl.Waypoints(end,:),xy,grid2world(exploreMap,coord_obstacle_cells));
                if (viable == 0) % It is evaluated with positions because viablepath works with world coordinates
                    continue;
                end
            for w = 1:length(coord_sensed_cells) % with all cells sensed
                agregar = true;
                if (poss_poses_A(k,1) == coord_sensed_cells(w,1) && poss_poses_A(k,2) == coord_sensed_cells(w,2)) % if that cell has already been explored break
                    agregar = false;
                    break;    
                end
            end
            if (agregar == true)
                i = i + 1;
                poss_poses_B(i,:) = poss_poses_A(k,:); % The coordinates and angle of the new possible waypoint are added
            end
        end
        if (i == 1) % If there is only one possible pose it is direct
            new_waypoint = grid2world(exploreMap,poss_poses_B(1:2)); % It is passed to world coordinates to add to the controller
        end        
        % if there are several possible poses: decide which direction of the possible ones to go (of the fan)
        M = mean(poss_poses_B);
        if (M ~= 0 & i >= 2) % If it is 0 it is because the end of the scan has been reached and there are no cells with a detected NaN range that have not been scanned
            mean_angle = M(3);   % It is directed in the mean of all angles
            for k = 1:length(poss_poses_B) % each possible position is compared
                if (poss_poses_B(k,3) >= mean_angle) % If the angle evaluated exceeds the average, it is considered adequate (it starts from negative angles)
                    new_waypoint = grid2world(exploreMap,poss_poses_B(k,1:2)); % It is passed to world coordinates to add to the controller
                    break;    
                end
            end
        end
        if (new_waypoint ~= 0) % if it is equal to 0 (it is no longer possible to add waypoints, because the end of the explo is approaching)
            % Reducing the distance obtained by the sensor to generate a smoother scanning path
            if (new_waypoint(1) > (currPos(1)+1.5))
                new_waypoint(1) = new_waypoint(1) - 1;
            elseif (new_waypoint(1) < (currPos(1)-1.5))
                new_waypoint(1) = new_waypoint(1) + 1;
            end
            if (new_waypoint(2) > (currPos(2)+1.5))
                new_waypoint(2) = new_waypoint(2) - 1;
            elseif (new_waypoint(2) < (currPos(2)-1.5)) % margin sacurity
                new_waypoint(2) = new_waypoint(2) + 1;
            end 
            ppControl.Waypoints(waypoints_idx,:) = [new_waypoint(1), new_waypoint(2)];
            waypoints_idx = waypoints_idx + 1;            
            new_waypoint = 0;
        end
        display(ppControl.Waypoints)
    end
    
    
    % THE COORDINATES OF THE SENSED CELLS ARE SAVED
    for i = 1:length(positions)
        [endpts,midpts] = raycast(exploreMap,currPos,positions(i,:)); % returns the points scanned by the laser in grid coordinates (rows, columns)
        setOccupancy(sensedMap,midpts,zeros(length(midpts),1),'grid'); % are added as free space to the sensing map
        for k = 1:length(midpts) % each midpoint is compared
            for w = 1:length(coord_sensed_cells) % with all cells sensed
                agregar = true;
                if (midpts(k,1) == coord_sensed_cells(w,1) && midpts(k,2) == coord_sensed_cells(w,2)) % if break has already been felt
                    agregar = false;
                    break;    
                end
            end
            if (agregar == true)
                index = index + 1;
                coord_sensed_cells(index,:) = midpts(k,:); 
            end
        end
    end   
    figure(3)
    show(sensedMap);
    title('Sensed cells');
    
    
    % CALCULATION OF THE % OF EXPLORED SURFACE
    occMatrix = checkOccupancy(sensedMap);
    num_sensed_cells = 0; 
    for c = 1:mapdimx 
        for r = 1:mapdimy 
            if (occMatrix(c, r) == 0)
                num_sensed_cells = num_sensed_cells + 1; % obtaining the number of sensed cells (free cells)
            end
        end
    end
    sup_explo = num_sensed_cells / num_free_cells; % Relationship between the sensed cells and the total number of free cells in the reference map

    
    % EXPLORATION END CONDITIONS
    if (sup_explo >= 0.998) % or a high % of exploration is reached
        disp("Exploration finished")
        last_pose = currPose; % The last position is returned for the origin navigation algorithm.
        break;
    end  
    dist = norm(ppControl.Waypoints(end,:)-currPos);
    if (dist < .2) % or the robot reaches the last generated waypoint (in the end, no more are generated because there are no unexplored cells)
        disp("Exploration finished")
        last_pose = currPose;
        break;
    end
    
    
    % CALCULATION OF THE DISTANCE TRAVELED
    if (idx > 1)
        prevPos = poses(1:2, idx-1)';
        dist_rec = dist_rec + norm(currPos-prevPos);
    end
    
    % Update visualization
    plotTrvec = [poses(1:2, idx+1); 0];
    plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
    
    % Delete image of the last robot to prevent displaying multiple robots
    if idx > 1
       items = get(ax1, 'Children');
       delete(items(1)); 
    end

    % Plot robot onto known map
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax1);
    % Plot robot on new map
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax2);

    % Wait to iterate at the proper rate
    waitfor(r);
end


% TIME ELAPSED IN GENERATING THE MAP
tiempo = toc;

end
