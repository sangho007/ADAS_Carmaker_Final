clear all; close all; clc;

%% Localization plotter

id_waypoint = 254; % The specific number will need verification

f1 = load('Logic/map/waypoints_data.mat');

n_waypoint = height(f1.ids); % Determine the number of IDs in the waypoints data

% Index identifying a specific road profile

id_roadprofile = 13;

f2 = load('Logic/map/roadprofiles_data.mat');

n_roadprofile = height(f2.ids); % Determine the number of IDs in the road profile data

disp(n_roadprofile)

figure(1);

% Plot the entire map
plot(f1.waypoints(:,1), f1.waypoints(:,2), 'k.'); % Plot all waypoints as black dots
hold on;

% Loop through each road profile
for m = 1 : n_roadprofile
    x = [f1.waypoints(f2.waypoints(m,1),1)];
    y = [f1.waypoints(f2.waypoints(m,1),2)];
    for n=2:50
        % If waypoint index is not 0 (meaning it's a valid waypoint)
        if f2.waypoints(m,n) ~= 0 
            x(end+1) = f1.waypoints(f2.waypoints(m,n),1);
            y(end+1) = f1.waypoints(f2.waypoints(m,n),2);
        end
    end
    
    % Color the current road profile red if selected, otherwise blue
    i_set1 = linspace(1,10,10);
    i_set2 = linspace(11,20,10);
    i_set3 = linspace(21,30,10);
    i_set4 = linspace(31,33,3);
    if ismember(m,i_set1)
        plot(x, y, 'r-','LineWidth',2 );
    elseif ismember(m,i_set2)
        plot(x, y, 'g-','LineWidth',2 );
    elseif ismember(m,i_set3)
        plot(x, y,'b-');   
    elseif ismember(m,i_set4)
        plot(x, y,'y-');
    end
end

xlim([-200 150]);
ylim([-200 50]);  
xlabel('X (m)');
ylabel('Y (m)');
title('Global Map');
grid on;

%% Animate Movement for Road Profile in path order

path = [1 2 3 4 5 16 17 28 31 33];
num_future_points = 20; % 미래 점들의 개수를 설정합니다.

figure(2);

for p = 1:length(path)
    i = path(p);
    
    % Extract the global X, Y coordinates for the current path
    p_x = [f1.waypoints(f2.waypoints(i,1),1)];
    p_y = [f1.waypoints(f2.waypoints(i,1),2)];
    for j=2:50
        if f2.waypoints(i,j) ~= 0 % Valid waypoint
            p_x(end+1) = f1.waypoints(f2.waypoints(i,j),1);
            p_y(end+1) = f1.waypoints(f2.waypoints(i,j),2);
        end
    end
    
    % Initialize for animation
    for k = 1:length(p_x)
        clf; % Clear the figure
        
        if k == 1
            % Initial orientation based on the first segment
            Yaw_ego = atan2((p_y(2) - p_y(1)), (p_x(2) - p_x(1)));
        else
            % Update orientation based on current segment
            Yaw_ego = atan2((p_y(k) - p_y(k-1)), (p_x(k) - p_x(k-1)));
        end
        
        % Create a Global2Local_class object for converting the entire map
        g2l_map = Global2Local_class(length(f1.waypoints));
        
        % Convert the entire map to local coordinates
        localPoints_map = g2l_map.convert([f1.waypoints(:,1), f1.waypoints(:,2)], Yaw_ego, p_x(k), p_y(k)).LocalPoints;
        
        % Plot the entire map in local coordinates
        plot(localPoints_map(:,1), localPoints_map(:,2), 'k.'); % Plot all waypoints as black dots
        hold on;
        
        % Loop through each road profile
        for m = 1 : n_roadprofile
            x = [f1.waypoints(f2.waypoints(m,1),1)];
            y = [f1.waypoints(f2.waypoints(m,1),2)];
            for n=2:50
                % If waypoint index is not 0 (meaning it's a valid waypoint)
                if f2.waypoints(m,n) ~= 0 
                    x(end+1) = f1.waypoints(f2.waypoints(m,n),1);
                    y(end+1) = f1.waypoints(f2.waypoints(m,n),2);
                end
            end
            
            % Create a Global2Local_class object for converting the current road profile
            g2l_profile = Global2Local_class(length(x));
            
            % Convert the current road profile to local coordinates
            localPoints_profile = g2l_profile.convert([x', y'], Yaw_ego, p_x(k), p_y(k)).LocalPoints;
            
            % Color the current road profile red if selected, otherwise blue
            if m == i
                plot(localPoints_profile(:,1), localPoints_profile(:,2), 'r-','LineWidth',2 );
            else
                plot(localPoints_profile(:,1), localPoints_profile(:,2), 'b-','LineWidth',1 );
            end
        end
        
        % Create a Global2Local_class object for converting the remaining global path
        g2l_path = Global2Local_class(min(num_future_points, length(p_x)-k) + 1);
        
        % Convert the remaining global path to local coordinates
        localPoints_path = g2l_path.convert([p_x(k:min(k+num_future_points, length(p_x)))', p_y(k:min(k+num_future_points, length(p_x)))'], Yaw_ego, p_x(k), p_y(k)).LocalPoints;
        
        % Animation: Plot the local path from the current position
        plot(localPoints_path(:,1), localPoints_path(:,2), 'c*','LineWidth',2);
        plot(0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Plot the vehicle position
        
        xlim([-100 100]);
        ylim([-100 100]);
        xlabel('X (m)');
        ylabel('Y (m)');
        title(['Road ID: ' num2str(i)]);
        grid on;
        
        pause(0.1); % Pause for animation effect
        
        % Check if the current position has reached the end of the current path
        if k == length(p_x)
            break; % Move to the next path
        end
    end
end