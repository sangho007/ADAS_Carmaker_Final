clear;close;clc;

% % Define path and start coordinates
% path = 1:10;
% start_x = 59.0003214677011;
% start_y = 24.2296986157458;
% 
% % Call path_animation function
% [current_road_id, localPoints_map] = Global2Local_path(path, start_x, start_y);
% 
% % Plot the entire map in local coordinates
% figure(1);
% plot(localPoints_map(:,1), localPoints_map(:,2), 'k.'); % Plot all waypoints as black dots
% hold on;
% 
% xlim([-100 100]);
% ylim([-100 100]);
% xlabel('X (m)');
% ylabel('Y (m)');
% title(['Current Road ID: ' num2str(current_road_id)]);
% grid on;

f1 = load('Logic/map/waypoints_data.mat');
f2 = load('Logic/map/roadprofiles_data.mat');


len_id = zeros(1,33)
for i = 1:33
    cnt = 0;
    for j = 1:50    
        if f2.waypoints(i,j) ~= 0
            cnt = cnt +1;
        end
    end
    len_id(1,i) = cnt;
end

disp(len_id)