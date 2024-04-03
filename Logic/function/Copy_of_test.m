%변수 불러오기 (실행)
run("GLOBAL_VAL_not.m");
data_load = load("waypoints_data.mat");
map_center = data_load.waypoints;

% map waypoints
mapx = map_center(:, 1);
mapy = map_center(:, 2);

% static obstacles
% obs = [3.0, WIDTH;
%        5, -WIDTH;
%        7, WIDTH;
%        8.5, -WIDTH];
% i = 24;
% j = 47;
% k = 12;
% l = 700;
% obs_global = [map_center(i,1), map_center(i,2);
%     map_center(j,1), map_center(j,2);
%     map_center(k,1), map_center(k,2);
%     map_center(l,1), map_center(l,2)];
num_obs = 10;
num_waypoints = size(map_center,1);
random_indices = randperm(num_waypoints, min(num_obs,num_waypoints));
obs_global = map_center(random_indices, :);
obs = zeros(length(obs_global),2);
for i = 1:length(obs_global)
    x = obs_global(i,1);
    y = obs_global(i,2);
    [frenet_obs_s, frenet_obs_d] = get_frenet(x, y, mapx, mapy); % get_frenet 함수의 MATLAB 버전 필요
    obs(i,1) = frenet_obs_s;
    obs(i,2) = frenet_obs_d;
end



% get maps
maps = zeros(size(mapx));
for i = 1:length(mapx)
    x = mapx(i);
    y = mapy(i);
    [sd, ~] = get_frenet(x, y, mapx, mapy); % get_frenet 함수의 MATLAB 버전 필요
    maps(i) = sd;
end

% get global position info. of static obstacles
% obs_global = zeros(size(obs));
% for i = 1:size(obs, 1)
%     t_s = obs(i, 1);
%     t_d = obs(i, 2);
%     [xy, ~, ~] = get_cartesian(t_s, t_d, mapx, mapy, maps); % get_cartesian 함수의 MATLAB 버전 필요
%     obs_global(i, :) = xy;
% end

% 자동차 관련 초기 조건
x = map_center(1,1);
y = map_center(1,2);
yaw = 0;%90 * pi / 180; % MATLAB에서는 pi를 직접 사용
v = 0.5;
a = 0;

[s, d] = get_frenet(x, y, mapx, mapy);
[x, y, yaw_road] = get_cartesian(s, d, mapx, mapy, maps);
yawi = yaw - yaw_road;

% s 방향 초기 조건
si = s;
si_d = v * cos(yawi);
si_dd = a * cos(yawi);
sf_d = TARGET_SPEED;
sf_dd = 0;

% d 방향 초기 조건
di = d;
di_d = v * sin(yawi);
di_dd = a * sin(yawi);
df_d = 0;
df_dd = 0;

opt_d = di;
figure(1)
hold on
plot(obs_global(:,1),obs_global(:,2),"*","Linewidth",2)
run("Localization_plot_fcn.m")
hold on
% 시뮬레이션 수행 (SIM_STEP 만큼)
for step = 1:SIM_STEP
    % optimal planning 수행 (output : valid path & optimal path index)
    [path, opt_ind] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d);

    % 다음 시뮬레이션 스텝에서 사용할 초기 조건 업데이트
    si = path(opt_ind).s(2);
    si_d = path(opt_ind).s_d(2);
    si_dd = path(opt_ind).s_dd(2);
    di = path(opt_ind).d(2);
    di_d = path(opt_ind).d_d(2);
    di_dd = path(opt_ind).d_dd(2);
    
    % consistency cost를 위해 업데이트
    opt_d = path(opt_ind).d(end);
    figure(1)
    plot(path(opt_ind).x, path(opt_ind).y , "Linewidth", 1);
    %     figure(2)
%     hold on
%     [frenet_s, frenet_d] = get_frenet(path{opt_ind}.x(1), path{opt_ind}.y(1), mapx, mapy);
%     plot(frenet_s, frenet_d ,'*', "Linewidth", 1);
%     for i = 1:length(path)
%         plot(path{i}.x, path{i}.y , '*')
%     end
    drawnow;
    % 그래픽과 시뮬레이션 업데이트 로직 필요
end



function next_wp = next_waypoint(x,y,mapx,mapy)
    closest_wp = get_closest_waypoints(x, y, mapx, mapy);
    map_vec = [mapx(closest_wp + 1) - mapx(closest_wp), mapy(closest_wp + 1) - mapy(closest_wp)];
    
    ego_vec = [x - mapx(closest_wp), y - mapy(closest_wp)];

    direction = sign(dot(map_vec, ego_vec));

    if direction >= 0
        next_wp = closest_wp + 1;
    else
        next_wp = closest_wp;
    end
end

function closest_wp = get_closest_waypoints(x, y, mapx, mapy)
    min_len = 1e10;
    closest_wp = 0;
    for i = 1:length(mapx)-1
        t_mapx = mapx(i);
        t_mapy = mapy(i);
        dist = get_dist(x, y, t_mapx, t_mapy);

        if dist < min_len
            min_len = dist;
            closest_wp = i;
        end
    end
end

function dist = get_dist(x1, y1, x2, y2)
    dist = sqrt((x1 - x2)^2 + (y1 - y2)^2);
end

function [frenet_s, frenet_d] = get_frenet(x, y, mapx, mapy)
    next_wp = next_waypoint(x, y, mapx, mapy);
    prev_wp = next_wp -1;

    n_x = mapx(next_wp) - mapx(prev_wp);
    n_y = mapy(next_wp) - mapy(prev_wp);
    x_x = x - mapx(prev_wp);
    x_y = y - mapy(prev_wp);

    proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    proj_x = proj_norm*n_x;
    proj_y = proj_norm*n_y;

    frenet_d = get_dist(x_x, x_y, proj_x, proj_y);

    ego_vec = [x-mapx(prev_wp), y-mapy(prev_wp), 0];
    map_vec = [n_x, n_y, 0];
    d_cross = cross(ego_vec,map_vec);
    if d_cross(end) > 0
        frenet_d = -frenet_d;
    end
    frenet_s = 0;
    for i=1:prev_wp 
        frenet_s = frenet_s + get_dist(mapx(i),mapy(i),mapx(i+1),mapy(i+1));
    end
    frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y);
end

function [x, y, heading] = get_cartesian(s, d, mapx, mapy, maps)
    prev_wp = 1;

    s = mod(s, maps(end-1));

    while (s > maps(prev_wp+1)) && (prev_wp < length(maps)-2)
        prev_wp = prev_wp + 1;
    end

    next_wp = mod(prev_wp+1, length(mapx));
    if next_wp == 0
        next_wp = length(mapx);
    end

    dx = mapx(next_wp) - mapx(prev_wp);
    dy = mapy(next_wp) - mapy(prev_wp);

    heading = atan2(dy, dx); % [rad]

    % the x,y,s along the segment
    seg_s = s - maps(prev_wp);

    seg_x = mapx(prev_wp) + seg_s * cos(heading);
    seg_y = mapy(prev_wp) + seg_s * sin(heading);

    perp_heading = heading + 90 * pi / 180;
    x = seg_x + d * cos(perp_heading);
    y = seg_y + d * sin(perp_heading);
end

% function frenet_paths = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
%     run("GLOBAL_VAL_not.m");
%     %frenet_paths = cell(length(DF_SET)*length(MIN_T:DT_T:MAX_T),1);%final value may has zero so you can check value
%     num_paths = length(DF_SET)*length(MIN_T:DT_T:MAX_T);
%     frenet_paths(num_paths) = FrenetPath();
%     i = 1;
%     for df = 1:length(DF_SET)
%         for T = MIN_T:DT_T:MAX_T
%             fp = FrenetPath();
%             lat_traj = QuinticPolynomial(di, di_d, di_dd, DF_SET(df), df_d, df_dd, T); %횡방향
%             
%             fp.t = 0:DT:T-DT;
%             fp.d = arrayfun(@(t) calc_pos(lat_traj, t), fp.t);
%             fp.d_d = arrayfun(@(t) calc_vel(lat_traj, t), fp.t);
%             fp.d_dd = arrayfun(@(t) calc_acc(lat_traj, t), fp.t);
%             fp.d_ddd = arrayfun(@(t) calc_jerk(lat_traj, t), fp.t);
%             
%             % Deep copy fp to tfp
%             tfp = fp; % Assume 'copy' is a function that copies FrenetPath object
%             lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T); %종방향
%             
%             tfp.s = arrayfun(@(t) calc_pos(lon_traj, t), fp.t);
%             tfp.s_d = arrayfun(@(t) calc_vel(lon_traj, t), fp.t);
%             tfp.s_dd = arrayfun(@(t) calc_acc(lon_traj, t), fp.t);
%             tfp.s_ddd = arrayfun(@(t) calc_jerk(lon_traj, t), fp.t);
%             
%             % Extend the path if T < MAX_T
%             for t_t = T:DT:MAX_T-DT
%                 tfp.t(end+1) = t_t;
%                 tfp.d(end+1) = tfp.d(end);
%                 t_s = tfp.s(end) + tfp.s_d(end) * DT;
%                 tfp.s(end+1) = t_s;
%                 
%                 tfp.s_d(end+1) = tfp.s_d(end);
%                 tfp.s_dd(end+1) = tfp.s_dd(end);
%                 tfp.s_ddd(end+1) = tfp.s_ddd(end);
%                 
%                 tfp.d_d(end+1) = tfp.d_d(end);
%                 tfp.d_dd(end+1) = tfp.d_dd(end);
%                 tfp.d_ddd(end+1) = tfp.d_ddd(end);
%             end
%             
%             J_lat = sum(tfp.d_ddd.^2);
%             J_lon = sum(tfp.s_ddd.^2);
%             
%             d_diff = (tfp.d(end) - opt_d)^2;
%             v_diff = (TARGET_SPEED - tfp.s_d(end))^2;
%             
%             tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
%             tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
%             
%             tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon;
%             
%             frenet_paths(i) = tfp;
%             i = i + 1;
%         end
%     end
% end

function frenet_paths = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
    run("GLOBAL_VAL_not.m");
    num_paths = length(DF_SET) * length(MIN_T:DT_T:MAX_T);
    
    % 구조체 배열을 초기화합니다.
    frenet_paths(num_paths) = struct('t', [], 'd', [], 'd_d', [], 'd_dd', [], 'd_ddd', [], ...
                                     's', [], 's_d', [], 's_dd', [], 's_ddd', [], ...
                                     'c_lat', 0.0, 'c_lon', 0.0, 'c_tot', 0.0, ...
                                     'x', [], 'y', [], 'yaw', [], 'ds', [], 'kappa', []);
    
    i = 1;
    for df = 1:length(DF_SET)
        for T = MIN_T:DT_T:MAX_T
            % 각 경로에 대한 데이터를 계산합니다.
            lat_traj = QuinticPolynomial(di, di_d, di_dd, DF_SET(df), df_d, df_dd, T); %횡방향
            
            t = 0:DT:T-DT;
            d = arrayfun(@(t) calc_pos(lat_traj, t), t);
            d_d = arrayfun(@(t) calc_vel(lat_traj, t), t);
            d_dd = arrayfun(@(t) calc_acc(lat_traj, t), t);
            d_ddd = arrayfun(@(t) calc_jerk(lat_traj, t), t);
            
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T); %종방향
            
            s = arrayfun(@(t) calc_pos(lon_traj, t), t);
            s_d = arrayfun(@(t) calc_vel(lon_traj, t), t);
            s_dd = arrayfun(@(t) calc_acc(lon_traj, t), t);
            s_ddd = arrayfun(@(t) calc_jerk(lon_traj, t), t);
            
            % 경로 비용 계산
            J_lat = sum(d_ddd.^2);
            J_lon = sum(s_ddd.^2);
            d_diff = (d(end) - opt_d)^2;
            v_diff = (TARGET_SPEED - s_d(end))^2;
            c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
            c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
            c_tot = K_LAT * c_lat + K_LON * c_lon;
            
            % 구조체 배열에 데이터를 할당합니다.
            frenet_paths(i) = struct('t', t, 'd', d, 'd_d', d_d, 'd_dd', d_dd, 'd_ddd', d_ddd, ...
                                     's', s, 's_d', s_d, 's_dd', s_dd, 's_ddd', s_ddd, ...
                                     'c_lat', c_lat, 'c_lon', c_lon, 'c_tot', c_tot, ...
                                     'x', [], 'y', [], 'yaw', [], 'ds', [], 'kappa', []);
            i = i + 1;
        end
    end
end

function fplist = calc_global_paths(fplist, mapx, mapy, maps)
    for i = 1:length(fplist)
        fp = fplist(i);
        fp.x = zeros(length(fp.s),1);
        fp.y = zeros(length(fp.s),1);
        for j = 1:length(fp.s)
            t_s = fp.s(j);
            t_d = fp.d(j);
            [t_x, t_y, ~] = get_cartesian(t_s, t_d, mapx, mapy, maps);
            fp.x(j) = t_x;
            fp.y(j) = t_y;
        end
        
        fp.yaw = zeros(length(fp.x)-1,1);
        fp.ds = zeros(length(fp.x)-1,1);
        for j = 1:length(fp.x)-1
            dx = fp.x(j+1) - fp.x(j);
            dy = fp.y(j+1) - fp.y(j);
            fp.yaw(j) = atan2(dy, dx);
            fp.ds(j) = hypot(dx, dy);
        end
        
        fp.yaw(end+1) = fp.yaw(end);
        fp.ds(end+1) = fp.ds(end);
        
        for j = 1:length(fp.yaw)-1
            yaw_diff = fp.yaw(j+1) - fp.yaw(j);
            yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));
            fp.kappa(end+1) = yaw_diff / fp.ds(j);
        end
        
        fplist(i) = fp;
    end
end

function collision = collision_check(fp, obs, mapx, mapy, maps)
    run("GLOBAL_VAL_not.m");
    collision = false;
    for i = 1:length(obs)
        [obs_x,obs_y,~] = get_cartesian(obs(i, 1), obs(i, 2), mapx, mapy, maps);
        
        d = arrayfun(@(x, y) (x - obs_x)^2 + (y - obs_y)^2, fp.x, fp.y);
        if any(d <= COL_CHECK^2)
            collision = true;
            return;
        end
    end
end

function ok_paths = check_path(fplist, obs, mapx, mapy, maps)
    run("GLOBAL_VAL_not.m");
    ok_ind = zeros(length(fplist),1);
    for i = 1:length(fplist)
        fp = fplist(i);
        
        acc_squared = arrayfun(@(s_dd, d_dd) abs(s_dd^2 + d_dd^2), fp.s_dd, fp.d_dd);
        
        if any(fp.s_d > V_MAX) || ...
           any(acc_squared > ACC_MAX^2) || ...
           any(abs(fp.kappa) > K_MAX) || ...
           collision_check(fp, obs, mapx, mapy, maps)
            continue;
        else
            ok_ind(i) = i;
        end
    end
    ok_paths = fplist(ok_ind(ok_ind ~= 0));
    %ok_paths = fplist{ok_ind};
end

function [fplist, opt_ind] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d)
    fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d);
    fplist = calc_global_paths(fplist, mapx, mapy, maps);

    fplist = check_path(fplist, obs, mapx, mapy, maps);
    % Find minimum cost path
    min_cost = inf;
    %opt_traj = cell(length(fplist),1);
    opt_ind = 0;
    
    for i = 1:length(fplist)
        if min_cost >= fplist(i).c_tot
            min_cost = fplist(i).c_tot;
            %opt_traj(i) = fplist(i);
            opt_ind = i;
        end
    end

    % Check if a solution exists
    if opt_ind == 0
        disp('No solution!');
    end
end