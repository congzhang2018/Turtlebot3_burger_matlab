% Simple Obstacle Avoidance

%%% CONNECT TO TURTLEBOT FIRST!!!
if ismember('/scan', rostopic('list'))
    laser_sub = rossubscriber('/scan');
end
% Set some parameters that will be used in the processing loop.
% You can modify these values for different behavior.
spinVelocity = 0.6;       % Angular velocity (rad/s)
forwardVelocity = 0.1;    % Linear velocity (m/s)
backwardVelocity = -0.02; % Linear velocity (reverse) (m/s)
distanceThreshold = 0.6;  % Distance threshold (m) for turning

% Run a loop to move the robot forward and compute the closest obstacles to
% the robot. When an obstacle is within the limits of the distanceThreshold,
% the robot turns. This loop stops after 20 seconds of run time.
% CTRL+C (or Control+C on the Mac) also stops this loop.
tic;
while toc < 20
    % Collect information from laser scan
    scan_data = receive(laser_sub);
    plot(scan_data);
    data = readCartesian(scan_data);
    x = data(:,1);
    y = data(:,2);
    % Compute distance of the closest obstacle
    dist = sqrt(x.^2 + y.^2);
    minDist = min(dist);
    % Command robot action
    if minDist < distanceThreshold
        % If close to obstacle, back up slightly and spin
        velocity_msg.Angular.Z = spinVelocity;
        velocity_msg.Linear.X = backwardVelocity;
    else
        % Continue on forward path
        velocity_msg.Linear.X = forwardVelocity;
        velocity_msg.Angular.Z = 0;
    end
    send(robot_pub,velocity_msg);
end

