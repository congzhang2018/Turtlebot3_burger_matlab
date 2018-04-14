function GoTo(Target, robot_pub, imu_sub)
% controller parameters
parameters.Krho = 0.5;
parameters.Kalpha = 1.5;
parameters.Kbeta = -0.6;
parameters.Ktheta = 0.1;
parameters.backwardAllowed = true;
parameters.useConstantSpeed = false;
parameters.constantSpeed = 0.8;

[robot_Rotation] = get_imu_data(imu_sub);
% current robot position and orientation
x = 0;
y = 0;
theta = robot_Rotation(1);

% goal position and orientation
xg = Target(1);
yg = Target(2);
thetag = Target(3);

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

velocity_msg = rosmessage(robot_pub);
% assign linear velocity
velocity_msg.Linear.X = vu;
velocity_msg.Linear.Y = 0;
velocity_msg.Linear.Z = 0;
% assign angular velocity
velocity_msg.Angular.X = 0;
velocity_msg.Angular.Y = 0;
velocity_msg.Angular.Z = omega;
% send this velocity command to robotw
send(robot_pub,velocity_msg);

end