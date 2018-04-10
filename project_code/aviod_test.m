
while(1)
    [x_lidar, y_lidar, scan_data] = get_lidar_data();
%     dist = sqrt(x_lidar.^2 + y_lidar.^2);
    dist = scan_data.Ranges(45:135);
    dist(dist == 0)=[];
    minDist = min(dist);
    if minDist < 0.5
        velocity_msg = rosmessage(robot_pub);
        % Command robot action
        velocity_msg.Angular.Z = 0;
        velocity_msg.Linear.X = 0.02;
        send_msgs(velocity_msg);
        disp("go back!!");
        plot(y_lidar, x_lidar, 'o');
    else
        velocity_msg = rosmessage(robot_pub);
        % Command robot action
        velocity_msg.Angular.Z = 0;
        velocity_msg.Linear.X = 0.02;
        send_msgs(velocity_msg);
        disp("go forward!!");
        plot(y_lidar, x_lidar, 'o');
    end
end