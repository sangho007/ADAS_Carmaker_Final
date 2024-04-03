function [resultx,resulty] = Copy_of_Trejectory_function(obs_global,global_x,global_y,global_v,global_a)
%변수 불러오기 (실행)
    [~,~,~,TARGET_SPEED,~,~,...
    ~,~,~,~,~,~,~,~,~,~,...
    ~,~] = GLOBAL_VAL();
    data_load = load("waypoints_data.mat");
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
    yaw = 0;%90 * pi / 180; % MATLAB에서는 pi를 직접 사용
    v = global_v;
    a = global_a;
    
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
    % optimal planning 수행 (output : valid path & optimal path index)
    [pathx, pathy] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d);
    resultx = pathx;                           
    resulty = pathy;
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



function [pathx,pathy] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d)
    [V_MAX,ACC_MAX,K_MAX,TARGET_SPEED,~,COL_CHECK,...
    MIN_T,MAX_T,DT_T,DT,K_J,K_T,K_D,K_V,K_LAT,K_LON,...
    ~,DF_SET] = GLOBAL_VAL(); 
    %fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d);
    cnt = 1;
    min_cost = inf;
    for df = 1:length(DF_SET)
        for T = MIN_T:DT_T:MAX_T
            % 각 경로에 대한 데이터를 계산합니다.
            lat_traj = QuinticPolynomial(di, di_d, di_dd, DF_SET(df), df_d, df_dd, T); %횡방향
            
            t = 0:DT:MAX_T;
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
            
        end
    %fplist = calc_global_paths(fplist, mapx, mapy, maps);
            x = zeros(length(s),1);
            y = zeros(length(s),1);
            for j = 1:length(s)
                t_s = s(j);
                t_d = d(j);
                [t_x, t_y, ~] = get_cartesian(t_s, t_d, mapx, mapy, maps);
                x(j) = t_x;
                y(j) = t_y;
            end
    
            yaw = zeros(length(s),1);
            ds = zeros(length(s),1);
            for j = 1:length(s)-1
                dx = x(j+1) - x(j);
                dy = y(j+1) - y(j);
                yaw(j) = atan2(dy, dx);
                ds(j) = hypot(dx, dy);
            end
            
            yaw(length(yaw)) = yaw(length(yaw)-1);
            ds(length(ds)) = ds(length(ds)-1);
            kappa = zeros(length(yaw)-1,1);
            for j = 1:length(yaw)-1
                yaw_diff = yaw(j+1) - yaw(j);
                yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));
                kappa(j) = yaw_diff / ds(j);
            end


        acc_squared = arrayfun(@(s_dd, d_dd) abs(s_dd^2 + d_dd^2), s_dd, d_dd);
        collision = false;
        for i = 1:length(obs)
            [obs_x,obs_y,~] = get_cartesian(obs(i, 1), obs(i, 2), mapx, mapy, maps);
            
            if any(arrayfun(@(x, y) (x - obs_x)^2 + (y - obs_y)^2, x, y) <= COL_CHECK^2)
                collision = true;
            end
        end
        if any(s_d > V_MAX) || ...
           any(acc_squared > ACC_MAX^2) || ...
           any(abs(kappa) > K_MAX) || ...
           collision
            pathx = zeros(length(s),1);
            pathy = zeros(length(s),1);
           continue;
        else
            if min_cost >= c_tot
                min_cost = c_tot;
                pathx = x;
                pathy = y;
            else
                pathx = zeros(length(s),1);
                pathy = zeros(length(s),1);
            end
        end
        cnt = cnt+1;
    end
end
