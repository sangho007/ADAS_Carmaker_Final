%% Localization plotter



f1 = load('Logic/map/waypoints_data.mat');

n_waypoint = height(f1.ids); % Determine the number of IDs in the waypoints data

% Index identifying a specific road profile

f2 = load('Logic/map/roadprofiles_data.mat');

n_roadprofile = height(f2.ids); % Determine the number of IDs in the road profile data

global_lane1 = zeros(n_roadprofile*50, 2);
global_lane2 = zeros(n_roadprofile*50, 2);
global_lane3 = zeros(n_roadprofile*50, 2);
global_lane4 = zeros(n_roadprofile*50, 2);

lane1_idx = 1;
lane2_idx = 1;
lane3_idx = 1; 
lane4_idx = 1;

% Plot
x = zeros(1, 50);
y = zeros(1, 50);

% Loop through each road profile
for i = 1 : n_roadprofile
    x(1) = f1.waypoints(f2.waypoints(i,1),1);
    y(1) = f1.waypoints(f2.waypoints(i,1),2);
    idx = 2;
    
    for j = 2:50
        % If waypoint index is not 0 (meaning it's a valid waypoint)
        if f2.waypoints(i,j) ~= 0
            x(idx) = f1.waypoints(f2.waypoints(i,j),1);
            y(idx) = f1.waypoints(f2.waypoints(i,j),2);
            idx = idx + 1;
        end
    end
    
    % Color the current road profile red if selected, otherwise blue
    i_set1 = 1:10;
    i_set2 = 11:20;
    i_set3 = 21:30;
    i_set4 = 31:33;
    
    if ismember(i,i_set1)
        global_lane1(lane1_idx:lane1_idx+idx-2,:) = [x(1:idx-1)', y(1:idx-1)'];
        lane1_idx = lane1_idx + idx - 1;
    elseif ismember(i,i_set2)
        global_lane2(lane2_idx:lane2_idx+idx-2,:) = [x(1:idx-1)', y(1:idx-1)'];
        lane2_idx = lane2_idx + idx - 1;
    elseif ismember(i,i_set3)
        global_lane3(lane3_idx:lane3_idx+idx-2,:) = [x(1:idx-1)', y(1:idx-1)'];
        lane3_idx = lane3_idx + idx - 1;
    elseif ismember(i,i_set4)
        global_lane4(lane4_idx:lane4_idx+idx-2,:) = [x(1:idx-1)', y(1:idx-1)'];
        lane4_idx = lane4_idx + idx - 1;
    end
    
end

Yaw_ego = 0.1;
g2l_map = Global2Local_class(length(global_lane2));

% Convert the entire map to local coordinates
localPoints_map = g2l_map.convert(global_lane2, Yaw_ego, -84, 24).LocalPoints;

figure(1)
scatter(localPoints_map(:,1),localPoints_map(:,2))
xlim([-200 200])
ylim([-200 200])
grid on

