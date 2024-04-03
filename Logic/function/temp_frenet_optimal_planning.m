function [pathx,pathy,opt_ind] = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d)
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
            
            t = 0:DT:T;
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
    
            yaw = zeros(length(x)-1,1);
            ds = zeros(length(x)-1,1);
            for j = 1:length(x)-1
                dx = fp.x(j+1) - x(j);
                dy = fp.y(j+1) - y(j);
                yaw(j) = atan2(dy, dx);
                ds(j) = hypot(dx, dy);
            end
            
            yaw(length(fp.yaw)+1) = yaw(length(fp.yaw));
            ds(length(fp.ds)+1) = ds(length(fp.ds));
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
            
            d = arrayfun(@(x, y) (x - obs_x)^2 + (y - obs_y)^2, x, y);
            if any(d <= COL_CHECK^2)
                collision = true;
            end
        end
        if any(fp.s_d > V_MAX) || ...
           any(acc_squared > ACC_MAX^2) || ...
           any(abs(fp.kappa) > K_MAX) || ...
           collision
           continue;
        else
            if min_cost >= c_tot
                min_cost = c_tot;
                pathx = x;
                pathy = y;
                opt_ind = cnt;
            end
        end
        cnt = cnt+1;
    end
end
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
        disp('No solution!');
    end
end
