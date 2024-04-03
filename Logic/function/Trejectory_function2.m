

% Trejectory_function.m
function [frenet_path_s, frenet_path_d, frenet_path_s_d, frenet_path_s_dd,opt_d] = Trejectory_function2(obs_global, global_x, global_y, local_v, local_a, local_yaw,opt_d_prev)
    % 변수 불러오기 (실행)
    data_load = load("waypoints_data.mat");
    map_center = data_load.waypoints;
    
    % map waypoints
    mapx = map_center(:, 1);
    mapy = map_center(:, 2);

    % 자동차 관련 초기 조건
    x = global_x;
    y = global_y;
    yaw = local_yaw;
    v = local_v;
    a = local_a;
    
    [s, d] = get_frenet(x, y, mapx, mapy);
    yawi = yaw; %- yaw_road;

    obs = zeros(length(obs_global),2);
    for i = 1:length(obs_global)
        x = obs_global(i,1);
        y = obs_global(i,2);
        [frenet_obs_s, frenet_obs_d] = get_frenet(x, y, mapx, mapy); % get_frenet 함수의 MATLAB 버전 필요
        obs(i,1) = frenet_obs_s;
        obs(i,2) = frenet_obs_d;
    end

    % s 방향 초기 조건
    si = s;
    si_d = v * cos(yawi);
    si_dd = a * cos(yawi);
    sf_d = local_v;
    sf_dd = 0;
    
    % d 방향 초기 조건
    di = d;
    di_d = v * sin(yawi);
    di_dd = a * sin(yawi);
    df_d = 0;
    df_dd = 0;
    
    opt_d = opt_d_prev;
    
    % optimal planning 수행 (output : valid path & optimal path index)
    [path, opt_ind] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, opt_d);
    
    % 경로 및 종 방향 가속도 출력
    frenet_path_s = path(opt_ind).s;
    frenet_path_d = path(opt_ind).d;
    frenet_path_s_d = path(opt_ind).s_d;
    frenet_path_s_dd = path(opt_ind).s_dd;
    opt_d = path(opt_ind).d(end);
end

% GLOBAL_VAL.m
function [V_MAX, ACC_MAX, K_MAX, LANE_WIDTH, COL_CHECK, ...
          MIN_T, MAX_T, DT_T, DT, K_J, K_T, K_D, K_V, K_LAT, K_LON, ...
          DF_SET] = GLOBAL_VAL()
    % initialize
    V_MAX = 30;      % maximum velocity [m/s]
    ACC_MAX = 9;    % maximum acceleration [m/ss]
    K_MAX = 900;      % maximum curvature [1/m]

    LANE_WIDTH = 3.5; % lane width [m]

    COL_CHECK = 3; % collision check distance [m]

    MIN_T = 1; % minimum terminal time [s]
    MAX_T = 2; % maximum terminal time [s]
    DT_T = 0.5; % dt for terminal time [s]
    DT = 0.1; % timestep for update

    % cost weights
    K_J = 0.1; % weight for jerk
    K_T = 0.1; % weight for terminal time
    K_D = 1.0; % weight for consistency
    K_V = 1.0; % weight for getting to target speed
    K_LAT = 1.0; % weight for lateral direction
    K_LON = 1.0; % weight for longitudinal direction


    % lateral planning 시 terminal position condition 후보 (양 차선 중앙)
    DF_SET = [-LANE_WIDTH/2,0, LANE_WIDTH/2];
end

% frenet_optimal_planning.m
function [fplist, opt_ind] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, opt_d)
    fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d);
    fplist = check_path(fplist, obs);
    
    % Find minimum cost path
    min_cost = inf;
    opt_ind = 0;
    
    for i = 1:length(fplist)
        if min_cost >= fplist(i).c_tot
            min_cost = fplist(i).c_tot;
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
        fp = struct('t', t, 'd', d, 'd_d', d_d, 'd_dd', d_dd, 'd_ddd', d_ddd, ...
                    's', s, 's_d', s_d, 's_dd', s_dd, 's_ddd', s_ddd, ...
                    'c_lat', 0, 'c_lon', 0, 'c_tot', 0);
        
        fplist = repmat(fp, 1, 1);
        opt_ind = 1;
    end
end

% calc_frenet_paths.m
function frenet_paths = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
    [~, ~, ~, ~, ~, ...
     MIN_T, MAX_T, DT_T, DT, K_J, K_T, K_D, K_V, K_LAT, K_LON, ...
     DF_SET] = GLOBAL_VAL();
    
    frenet_paths = repmat(struct('t', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 'd', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 'd_d', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 'd_dd', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 'd_ddd', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 's', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 's_d', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 's_dd', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 's_ddd', zeros(1, round((MAX_T-DT)/DT)+1), ...
                                 'c_lat', 0, 'c_lon', 0, 'c_tot', 0), ...
                          1, length(DF_SET)*round((MAX_T-MIN_T)/DT_T)+1);
    
    path_ind = 1;
    for df = DF_SET
        for T = MIN_T:DT_T:MAX_T
            % Lateral motion planning
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T);
            
            frenet_paths(path_ind).t = 0:DT:T;
            frenet_paths(path_ind).d = lat_traj.calc_pos(frenet_paths(path_ind).t);
            frenet_paths(path_ind).d_d = lat_traj.calc_vel(frenet_paths(path_ind).t);
            frenet_paths(path_ind).d_dd = lat_traj.calc_acc(frenet_paths(path_ind).t);
            frenet_paths(path_ind).d_ddd = lat_traj.calc_jerk(frenet_paths(path_ind).t);
            
            % Longitudinal motion planning (velocity keeping)
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T);
            
            frenet_paths(path_ind).s = lon_traj.calc_pos(frenet_paths(path_ind).t);
            frenet_paths(path_ind).s_d = lon_traj.calc_vel(frenet_paths(path_ind).t);
            frenet_paths(path_ind).s_dd = lon_traj.calc_acc(frenet_paths(path_ind).t);
            frenet_paths(path_ind).s_ddd = lon_traj.calc_jerk(frenet_paths(path_ind).t);
            
            % Cost calculation
            J_lat = sum(frenet_paths(path_ind).d_ddd.^2);
            J_lon = sum(frenet_paths(path_ind).s_ddd.^2);
            
            d_diff = (frenet_paths(path_ind).d(end) - opt_d)^2;
            v_diff = (sf_d - frenet_paths(path_ind).s_d(end))^2;
            
            frenet_paths(path_ind).c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
            frenet_paths(path_ind).c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
            frenet_paths(path_ind).c_tot = K_LAT * frenet_paths(path_ind).c_lat + K_LON * frenet_paths(path_ind).c_lon;
            
            path_ind = path_ind + 1;
        end
    end
end

% check_path.m
function ok_paths = check_path(fplist, obs)
    [V_MAX, ACC_MAX, K_MAX, ~, ~, ...
     ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ...
    ~] = GLOBAL_VAL();
    
    ok_paths = fplist;
    
    for i = 1:length(fplist)
        fp = fplist(i);
        
        if any(fp.s_d > V_MAX)
            ok_paths(i) = [];
            continue;
        end
        
        if any(fp.s_dd.^2 + fp.d_dd.^2 > ACC_MAX^2)
            ok_paths(i) = [];
            continue;
        end
        if any(abs(fp.kappa) > K_MAX)
            ok_paths(i) = [];
            continue;
        end
        if collision_check(fp, obs)
            ok_paths(i) = [];
            continue;
        end
    end
end

% collision_check.m
function collision = collision_check(fp, obs)
    [~, ~, ~, ~, ~, COL_CHECK, ...
     ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ...
     ~, ~] = GLOBAL_VAL();
    
    collision = false;
    
    for i = 1:length(obs)
        d = (fp.s - obs(i, 1)).^2 + (fp.d - obs(i, 2)).^2;
        
        if any(d <= COL_CHECK^2)
            collision = true;
            break;
        end
    end
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
    if d_cross(length(d_cross)) > 0
        frenet_d = -frenet_d;
    end
    frenet_s = 0;
    for i=1:prev_wp 
        frenet_s = frenet_s + get_dist(mapx(i),mapy(i),mapx(i+1),mapy(i+1));
    end
    frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y);
end

