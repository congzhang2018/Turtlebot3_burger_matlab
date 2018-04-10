function [velocity_msg, minDist] = aviod_object(x_lidar, y_lidar, robot_pub)
    dist = [];
    for i = 1:length(x_lidar)
        if x_lidar(i) > 0
            dist = [dist, sqrt(x_lidar(i)^2 + y_lidar(i)^2)];
        end
    end
    minDist = min(dist);
    velocity_msg = rosmessage(robot_pub);
    % Command robot action
    velocity_msg.Angular.Z = 0;
    velocity_msg.Linear.X = -0.02;
end