function [resultx,resulty,invalid...
    ,frenet_path_s, frenet_path_d, frenet_path_s_d, frenet_path_s_dd,opt_d_prev] ...
    = Trejectory_function(obs_global,global_x,global_y,global_v,global_a,global_yaw,opt_d_prev)
%변수 불러오기 (실행)
    data_load = load("Logic/map/waypoints_data.mat");
    map_center = data_load.waypoints;
    
    % map waypoints
    mapx = map_center(:, 1);
    mapy = map_center(:, 2);
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
    
    
    % 자동차 관련 초기 조건
    x = global_x;
    y = global_y;
    yaw = global_yaw;%90 * pi / 180; % MATLAB에서는 pi를 직접 사용
    v = global_v;
    a = global_a;
    
    [s, d] = get_frenet(x, y, mapx, mapy);
    [~, ~, yaw_road] = get_cartesian(s, d, mapx, mapy, maps);
    yawi = yaw - yaw_road;
    
    % s 방향 초기 조건
    si = s;
    si_d = v * cos(yawi);
    si_dd = a * cos(yawi);
    sf_d = global_v;
    sf_dd = 0;
    
    % d 방향 초기 조건
    di = d;
    di_d = v * sin(yawi);
    di_dd = a * sin(yawi);
    df_d = 0;
    df_dd = 0;
    persistent opt_d;
    if isempty(opt_d)
        opt_d = di;
    end
    % optimal planning 수행 (output : valid path & optimal path index)
    [path, opt_ind] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d);
%     figure(2)
%     hold on
%     for i = 1: length(path)
%         plot(path(i).x,path(i).y,'LineWidth',2)
%     end
    resultx = path(opt_ind).x;
    resulty = path(opt_ind).y;
    invalid = path(opt_ind).valid_id;
    frenet_path_s = path(opt_ind).s;
    frenet_path_d = path(opt_ind).d;
    frenet_path_s_d = path(opt_ind).s_d;
    frenet_path_s_dd = path(opt_ind).s_dd;
    opt_d = path(opt_ind).d(end);
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
    min_len = 10000000;
    closest_wp = 0;
    for i = 2:length(mapx)-1
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
    prev_wp = next_wp - 1;

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
    if d_cross(length(d_cross)) > 0
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

    s = mod(s, maps(length(maps)-1));

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

function frenet_paths = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
    [~, ~, ~, ~, ~, ~, ...
     MIN_T, MAX_T, DT_T, DT, K_J, K_T, K_D, K_V, K_LAT, K_LON, ...
     ~, DF_SET] = GLOBAL_VAL();
    
    max_path_length = round((MAX_T - DT) / DT + 1);
    num_paths = length(DF_SET) * round((MAX_T - MIN_T) / DT_T + 1);
    
    frenet_paths = repmat(struct('t', zeros(max_path_length,1), ...
                                 'd', zeros(1, max_path_length), ...
                                 'd_d', zeros(1, max_path_length), ...
                                 'd_dd', zeros(1, max_path_length), ...
                                 'd_ddd', zeros(1, max_path_length), ...
                                 's', zeros(1, max_path_length), ...
                                 's_d', zeros(1, max_path_length), ...
                                 's_dd', zeros(1, max_path_length), ...
                                 's_ddd', zeros(1, max_path_length), ...
                                 'x', zeros(max_path_length,1), ...
                                 'y', zeros(max_path_length,1), ...
                                 'yaw', zeros(max_path_length,1), ...
                                 'ds', zeros(max_path_length,1), ...
                                 'kappa', zeros(max_path_length,1), ...
                                 'c_lat', 0, 'c_lon', 0, 'c_tot', 0, ...
                                 'valid', false ,'valid_id', zeros(max_path_length,1))...
                                 , 1, num_paths);
    
    i = 1;
    for df = DF_SET
        for T = MIN_T:DT_T:MAX_T
            % 각 경로에 대한 데이터를 계산합니다.
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T);
            
            t = 0:DT:T-DT;
            d = arrayfun(@(t) lat_traj.calc_pos(t), t);
            d_d = arrayfun(@(t) lat_traj.calc_vel(t), t);
            d_dd = arrayfun(@(t) lat_traj.calc_acc(t), t);
            d_ddd = arrayfun(@(t) lat_traj.calc_jerk(t), t);
            
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T);
            
            s = arrayfun(@(t) lon_traj.calc_pos2(t), t);
            s_d = arrayfun(@(t) lon_traj.calc_vel2(t), t);
            s_dd = arrayfun(@(t) lon_traj.calc_acc2(t), t);
            s_ddd = arrayfun(@(t) lon_traj.calc_jerk2(t), t);
            
            % 경로 비용 계산
            J_lat = sum(d_ddd.^2);
            J_lon = sum(s_ddd.^2);
            d_diff = (d(end) - opt_d)^2;
            v_diff = (sf_d - s_d(end))^2;
            c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
            c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
            c_tot = K_LAT * c_lat + K_LON * c_lon;
            
            % 구조체 배열에 데이터를 할당합니다.
            path_length = length(t);
            frenet_paths(i).t(1:path_length) = t;
            frenet_paths(i).d(1:path_length) = d;
            frenet_paths(i).d_d(1:path_length) = d_d;
            frenet_paths(i).d_dd(1:path_length) = d_dd;
            frenet_paths(i).d_ddd(1:path_length) = d_ddd;
            frenet_paths(i).s(1:path_length) = s;
            frenet_paths(i).s_d(1:path_length) = s_d;
            frenet_paths(i).s_dd(1:path_length) = s_dd;
            frenet_paths(i).s_ddd(1:path_length) = s_ddd;
            frenet_paths(i).c_lat = c_lat;
            frenet_paths(i).c_lon = c_lon;
            frenet_paths(i).c_tot = c_tot;
            frenet_paths(i).valid = true;
            frenet_paths(i).valid_id(1:path_length) = 1:path_length;
            i = i + 1;
        end
    end
end

function fplist = calc_global_paths(fplist, mapx, mapy, maps)
    for i = 1:length(fplist)
        fp = fplist(i);
%         fp.x = zeros(length(fp.s),1);
%         fp.y = zeros(length(fp.s),1);
%         valid_indices = fp.valid_id(fp.valid_id > 0);
        for j = 1:length(fp.s)
        %for j = 1:valid_indices
            t_s = fp.s(j);
            t_d = fp.d(j);
            [t_x, t_y, ~] = get_cartesian(t_s, t_d, mapx, mapy, maps);
            fp.x(j) = t_x;
            fp.y(j) = t_y;
        end

%         fp.yaw = zeros(length(fp.x)-1,1);
%         fp.ds = zeros(length(fp.x)-1,1);
        for j = 1:length(fp.x)-1
        %for j = valid_indices(1:end-1)
            dx = fp.x(j+1) - fp.x(j);
            dy = fp.y(j+1) - fp.y(j);
            fp.yaw(j) = atan2(dy, dx);
            fp.ds(j) = hypot(dx, dy);
        end
        
        fp.yaw(length(fp.yaw)) = fp.yaw(length(fp.yaw)-1);
        fp.ds(length(fp.ds)) = fp.ds(length(fp.ds)-1);
%         fp.kappa = zeros(length(fp.yaw)-1,1);
        for j = 1:length(fp.yaw)-1
            yaw_diff = fp.yaw(j+1) - fp.yaw(j);
            yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));
            fp.kappa(j) = yaw_diff / fp.ds(j);
        end
%         fp.yaw(valid_indices(end)) = fp.yaw(valid_indices(end)-1);
%         fp.ds(valid_indices(end)) = fp.ds(valid_indices(end)-1);
%         
%         for j = valid_indices(1:end-1)
%             yaw_diff = fp.yaw(j+1) - fp.yaw(j);
%             yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));
%             fp.kappa(j) = yaw_diff / fp.ds(j);
%         end
        
        fplist(i).x = fp.x;
        fplist(i).y = fp.y;
        fplist(i).yaw = fp.yaw;
        fplist(i).ds = fp.ds;
        fplist(i).kappa = fp.kappa;
    end
end

function collision = collision_check(fp, obs, mapx, mapy, maps)
    [~,~,~,~,~,COL_CHECK,...
    ~,~,~,~,~,~,~,~,~,~,...
    ~,~] = GLOBAL_VAL();
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
    [V_MAX,ACC_MAX,K_MAX,~,~,~,...
    ~,~,~,~,~,~,~,~,~,~,...
    ~,~] = GLOBAL_VAL();
    ok_ind = zeros(length(fplist),1);
    for i = 1:length(fplist)
        fp = fplist(i);
        acc_squared = arrayfun(@(s_dd, d_dd) abs(s_dd^2 + d_dd^2), fp.s_dd, fp.d_dd);
        
%         if any(fp.s_d > V_MAX) || ...
%            any(acc_squared > ACC_MAX^2) || ...
%            any(abs(fp.kappa) > K_MAX) || ...
        if collision_check(fp, obs, mapx, mapy, maps)
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
            %opt_traj{i} = fplist(i);
            opt_ind = i;
        end
    end

    % Check if a solution exists
    if opt_ind == 0
        % Generate a path that maintains the current lane
        [~,~,~,~,~,~,...
        ~,MAX_T,~,DT,~,~,~,~,~,~,...
        ~,~] = GLOBAL_VAL();
        T = MAX_T;
        t = 0:DT:T-DT;
        path_length = length(t);
        
        % Lateral motion: maintain current lane
        d = ones(1, path_length) * di;
        d_d = zeros(1, path_length);
        d_dd = zeros(1, path_length);
        d_ddd = zeros(1, path_length);
        
        % Longitudinal motion: maintain current speed
        s = si + si_d * t;
        s_d = ones(1, path_length) * si_d;
        s_dd = zeros(1, path_length);
        s_ddd = zeros(1, path_length);
        
        % Create a new path structure
        fp = struct('t', t', 'd', d, 'd_d', d_d, 'd_dd', d_dd, 'd_ddd', d_ddd, ...
                    's', s, 's_d', s_d, 's_dd', s_dd, 's_ddd', s_ddd, ...
                    'x', zeros(path_length, 1), 'y', zeros(path_length, 1), ...
                    'yaw', zeros(path_length, 1), 'ds', zeros(path_length, 1), ...
                    'kappa', zeros(path_length, 1), ...
                    'c_lat', 0, 'c_lon', 0, 'c_tot', 0, 'valid', true...
                    ,'valid_id', zeros(path_length,1));
        
        fplist = repmat(fp, 1, 1);
        opt_ind = 1;
    end
end

function [left_boundary, right_boundary] = get_boundaries(d,start_lane) %start_lane = 2 
    [~,~,~,~,LANE_WIDTH,~,...
    ~,~,~,~,~,~,~,~,~,~,...
    ~,~] = GLOBAL_VAL();
    persistent prev_left_boundary prev_right_boundary
    
    if isempty(prev_left_boundary)
        % Initialize the boundaries if they are not set
        prev_left_boundary = d - LANE_WIDTH / 2;
        prev_right_boundary = d + LANE_WIDTH / 2;
    end
    
    % Calculate the boundary change
    left_boundary_change = d - LANE_WIDTH / 2 - prev_left_boundary;
    right_boundary_change = d + LANE_WIDTH / 2 - prev_right_boundary;
    
    % Update the boundaries
    left_boundary = prev_left_boundary + left_boundary_change;
    right_boundary = prev_right_boundary + right_boundary_change;
    
    % Store the current boundaries for the next iteration
    prev_left_boundary = left_boundary;
    prev_right_boundary = right_boundary;
end

