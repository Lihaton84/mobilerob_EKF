%% Setup
clear variables;
close all;

%% Parameters
virtualMachineIP = '192.168.8.117';
localIP = '192.168.70.1';
controllerType = 'purePursuit'; % 'purePursuit' or 'RezasController'
lookaheadDistance = 0.3; % lookahead distance for the controller
motionModelType = 'odometry'; % 'odometry' or 'velocity'
useRanges = true; % whether to use range measurements or not
filterType = 'ml-ekf'; % 'MonteCarlo' or 'EKFSLAM' or 'ML-EKF'

%% Load the given map and path
filePath = fullfile(fileparts(which(mfilename)), 'moroparams.mat');
load(filePath);

% Plot them
show(map);
hold on;
plot(path(:,1), path(:,2), 'ko-', 'linewidth', 2);

% Smooth the path
% smoothPath contains
%   smoothPath.qd     - points
%   smoothPath.qd_dot - velocities
%   smoothPath.time   - times
%smoothPath = path2smoothPath(path);

% Plot the smoothed path
%plot(smoothPath.qd(1,:), smoothPath.qd(2,:), 'r-', 'linewidth', 2);

%% Connect to the TurtleBot in Gazebo
rosshutdown % make sure there's no existing matlab ROS node
setenv('ROS_IP', localIP);
rosinit(virtualMachineIP);

% Create ROS subscribers for retrieving odometry measurements
% and true model states from Gazebo
odomSub = rossubscriber('odom', @odomCallback);
modelSub = rossubscriber('gazebo/model_states', @stateCallback);
global odomPose;
global gazeboPose;

% Create robot controller
%pathFollower = RobotController('path', smoothPath, ...
 %                              'controllerType', controllerType, ...
  %                             'lookaheadDistance', lookaheadDistance);

% Create a scanner object for retrieving landmark measurements
landmarkScanner = MoroLandmarkScanner();

% Create the filter
if useRanges
    nInputs = 3;
else
    nInputs = 2;
end
switch upper(filterType)
    case 'MONTECARLO'
        filter = MonteCarloParticleFilter(...
            'nInputs', nInputs, ...
            'motionModelType', motionModelType);
    case 'EKFSLAM'
        useRanges = true;
        filter = SimultaneousLocalizationAndMapping(...
            'motionModelType', motionModelType);
    case 'ML-EKF'
        filter = ExtendedKalmanFilter(...
            'nInputs', nInputs, ...
            'motionModelType', motionModelType, ...
            'knownCorrespondences', false);
end

%% Run the main loop until the pathFollower has finished.
% Set up variables for storing data
truePose = zeros(0,3);
odometryPose = zeros(0,3);
estimatedPose = zeros(0,3);
estimatedCovariance = zeros(0,3,3);

% Handles for plots
filterHandle = [];
trueHandle = [];
measurementHandle = zeros(0,3);

% Initial values
v = 0;
w = 0;
dt = 0;

% Loop iteration counter
i = 0;

while ~isDone(pathFollower)
    startLoop = tic;

    % Receive landmark measurement and odometry message.
    [angles, ranges] = landmarkScanner.scanForLandmarks();

    % Update estimated robot pose and covariance using new control and
    % sensor readings.
    sensorReadings = {angles};
    if useRanges
        sensorReadings{2} = ranges;
    end
    switch upper(motionModelType)
        case 'ODOMETRY'
            control = odomPose;
        case 'VELOCITY'
            control = [v;w;dt];
    end
    [poseEstimate, covarianceEstimate] = filter(control,sensorReadings{:});

    % Calculate and publish the robot control
    [v,w] = pathFollower(poseEstimate(1:3));

    % Plot the robot's estimated pose on the map.
    % Delete previous plot
    delete(filterHandle);
    delete(trueHandle);
    delete(measurementHandle);

    % Plot new iteration
    filterHandle = plot(filter);

    % Plot measurements
    measurementHandle = zeros(3,0);
    for channel = 1:3
        if ~isnan(angles(channel))
            color = zeros(3,1);
            color(channel) = 1;
            x = poseEstimate(1);
            y = poseEstimate(2);
            theta = poseEstimate(3);
            [dx, dy] = pol2cart(angles(channel)+theta, ranges(channel));
            measurementHandle(channel) = plot(x+dx, y+dy, 'o', ...
                                              'markerfacecolor', color, ...
                                              'color', color);
        end
    end

    % Plot robots true pose
    trueHandle = plotRobot(gazeboPose);
    drawnow;

    % Store data
    i = i + 1;
    truePose(i,:) = gazeboPose;
    odometryPose(i,:) = odomPose;
    estimatedPose(i,:) = poseEstimate(1:3);
    estimatedCovariance(i,:,:) = covarianceEstimate(1:3,1:3);

    endLoop = toc(startLoop);
    dt = max(endLoop, 0.1);
    pause(0.1-endLoop);
end

% Plot robot trajectories
trajectoryHandle = plot(truePose(:,1), truePose(:,2), 'r', ...
                        estimatedPose(:,1), estimatedPose(:,2), 'g', ...
                        odometryPose(:,1), odometryPose(:,2), 'b', ...
                        'linewidth', 2);
legend(trajectoryHandle, {'True','Estimated','Odometry'}, ...
       'location', 'northeastoutside')

%% Shutdown matlab global node
clear global
delete(odomSub)
delete(modelSub)
delete(landmarkScanner)
delete(pathFollower)
delete(filter)

rosshutdown
