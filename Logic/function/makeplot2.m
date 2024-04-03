obs_global = [10, 2; 20, 6; 30, -3; 40, 1; 50, 4]; % 장애물 정보 (예시)
global_x = 0; % 차량의 현재 x 좌표
global_y = 0; % 차량의 현재 y 좌표
global_v = 10; % 차량의 현재 속도
global_a = 0; % 차량의 현재 가속도
global_yaw = 0; % 차량의 현재 yaw 각도 (rad)

% Trejectory_function 호출
[resultx, resulty, invalid] = Trejectory_function(obs_global, global_x, global_y, global_v, global_a, global_yaw);

% 결과 출력
disp('Resultx:');
disp(resultx);
disp('Resulty:');
disp(resulty);
disp('Invalid:');
disp(invalid);

% 경로 그리기
figure(1);
plot(resultx, resulty, 'b-', 'LineWidth', 2);
hold on;
plot(resultx(invalid > 0), resulty(invalid > 0), 'ro', 'MarkerSize', 8);
xlabel('X');
ylabel('Y');
title('Trajectory');
legend('Path', 'Invalid Points');
grid on;