%% Connect to Turtlebot
% Connect to an External ROS Master
ip_robot = '192.168.0.116';   % ip address of robot, replace this one with yours
rosinit(ip_robot, 'NodeHost','192.168.0.117')
% check nodes running currently
rosnode list
% check topics that available
rostopic list
% check detial infomation about particular topic
topic_name = '/raspicam_node/image/compressed';
rostopic info topic_name

%% Read sensor data

%%% read lidar data
if ismember('/scan', rostopic('list'))
    laser_sub = rossubscriber('/scan');
end
scan_data = receive(laser_sub);

% plot laser data
tic;
  while toc < 100
      sscan_datacan = receive(laser_sub,3);
      plot(sscan_datacan);
  end

%%% read images
% if images captured by Pi camera.
if ismember('/raspicam_node/image/compressed', rostopic('list'))
    image_sub = rossubscriber('/raspicam_node/image/compressed');
end

% if you are using Gazebo, the topic list is different.
pi_cam_node = '/raspicam_node/image/compressed';
if ismember(pi_cam_node, rostopic('list'))
    image_sub = rossubscriber(pi_cam_node);
end
image_compressed = receive(image_sub);
image_compressed.Format = 'bgr8; jpeg compressed bgr8';
figure
imshow(readImage(image_compressed));
% to display a continuously updating image from Pi camera
tic;
  while toc < 20
      image_compressed = receive(image_sub);
      image_compressed.Format = 'bgr8; jpeg compressed bgr8';
      imshow(readImage(image_compressed))
  end
lines = detect_straight_lines(readImage(image_compressed));
%% Move the Robot
% controller parameters
parameters.Krho = 0.5;
parameters.Kalpha = 1.5;
parameters.Kbeta = -0.6;
parameters.Ktheta = 0.1;
parameters.backwardAllowed = true;
parameters.useConstantSpeed = false;
parameters.constantSpeed = 0.8;

% current robot position and orientation
x = 0;
y = 0;
theta = 0;

% goal position and orientation
xg = 0;
yg = 0;
thetag = 80/180*pi;

% compute control quantities
rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);     % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = lambda - theta;         % angle of the vector pointing from the robot to the goal in the robot frame
alpha = normalizeAngle(alpha);

beta = -lambda;
omega = parameters.Kalpha * alpha + parameters.Kbeta * beta + parameters.Ktheta * (thetag-theta); % [rad/s]
if parameters.useConstantSpeed
%     omega = parameters.constantSpeed/vu * omega;
    vu = parameters.constantSpeed;
    omega = parameters.constantSpeed/(parameters.Krho * rho) * omega;
else
    vu = parameters.Krho * rho; % [m/s]
end

linear_vel = [vu; 0; 0]; % meters per second
angular_vel = [0; 0; omega];  % radius per second

% Create a publisher for the /mobile_base/commands/velocity topic and the corresponding message containing the velocity values.
robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
velocity_msg = rosmessage(robot_pub);
% assign linear velocity
velocity_msg.Linear.X = 1;
velocity_msg.Linear.Y = 0;
velocity_msg.Linear.Z = 0;
% assign angular velocity
velocity_msg.Angular.X = 0;
velocity_msg.Angular.Y = 0;
velocity_msg.Angular.Z = 0;
% send this velocity command to robotw
send(robot_pub,velocity_msg);

%% Disconnect from the Robot

%%% clear workspace when you are finished with them
clear

%%% Shut down the global node and disconnect from the TurtleBot.
rosshutdown
