%% Parameters used by Auto Parking Valet using MPC and RL example

% Copyright 2020 The MathWorks, Inc.

%%

% Lidar parameters
maxLidarDist = 6;           % maximum distance that can be measured by lidar (m)
numSensors   = 12;          % number of lidar sensors
obsMat = map.ObstacleMatrix;
lidarTol    = 0.5;          % minimum distance measured by lidar (m)

% Error tolerances with target pose
xyerrTol    = 0.75;         % position error tolerance w.r.t. target pose (m)
terrTol     = deg2rad(10);  % orientation error tolerance w.r.t. target pose (m)

% Camera parameters
cameraDepth = 10;           % camera depth (m)
cameraViewAngle = deg2rad(120); % camera field of view (rad)

% Sample time, sim time
Ts = 0.1;                   % sample time of MPC and RL controllers (sec)
% Tv = 0.1;                 % sec
Tf = 50;                    % simulation time (sec)

% Parameters for training
speedMax = 2;               % maximum speed of ego vehicle (m/s)
steerMax = pi/4;            % maximum steering angle (rad)
trainXBounds  = map.TrainingZoneXLimits;
trainYBounds  = map.TrainingZoneYLimits;
trainTBounds  = [-2*pi 2*pi];

% Parameters for simulation
xBounds  = map.XLimits;
yBounds  = map.YLimits;
tBounds  = [-Inf Inf];