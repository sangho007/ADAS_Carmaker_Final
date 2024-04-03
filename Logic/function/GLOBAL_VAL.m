function [V_MAX,ACC_MAX,K_MAX,TARGET_SPEED,LANE_WIDTH,COL_CHECK,...
    MIN_T,MAX_T,DT_T,DT,K_J,K_T,K_D,K_V,K_LAT,K_LON,...
    SIM_STEP,DF_SET] = GLOBAL_VAL()
% GlobalVariables.m
% global V_MAX ACC_MAX K_MAX TARGET_SPEED LANE_WIDTH COL_CHECK
% global MIN_T MAX_T DT_T DT K_J K_T K_D K_V K_LAT K_LON SIM_STEP SHOW_ANIMATION
% global LENGTH WIDTH BACKTOWHEEL WHEEL_LEN WHEEL_WIDTH TREAD WB DF_SET
V_MAX = 30;      % maximum velocity [m/s]
ACC_MAX = 9;    % maximum acceleration [m/ss]
K_MAX = 10;      % maximum curvature [1/m]

TARGET_SPEED = 20/3.6; % target speed [m/s]
LANE_WIDTH = 3.5;  % lane width [m]

COL_CHECK = 2.5; % collision check distance [m]

MIN_T = 1; % minimum terminal time [s]
MAX_T = 2; % maximum terminal time [s]
DT_T = 0.1; % dt for terminal time [s]
DT = 0.2; % timestep for update

% cost weights
K_J = 0.1; % weight for jerk
K_T = 0.1; % weight for terminal time
K_D = 1.0; % weight for consistency
K_V = 1.0; % weight for getting to target speed
K_LAT = 0.8; % weight for lateral direction
K_LON = 1.0; % weight for longitudinal direction

SIM_STEP = 1000; % simulation step
%SHOW_ANIMATION = true; % plot 으로 결과 보여줄지 말지

% Vehicle parameters - plot 을 위한 파라미터
%LENGTH = 4.68;  % [m]
%WIDTH = 1.88;  % [m]
%BACKTOWHEEL = 0.1;  % [m]
%WHEEL_LEN = 0.03;  % [m]
%WHEEL_WIDTH = 0.02;  % [m]
%TREAD = 0.07;  % [m]
%WB = 0.22;  % [m]

% lateral planning 시 terminal position condition 후보 (양 차선 중앙)
DF_SET = [-LANE_WIDTH 0 LANE_WIDTH ];