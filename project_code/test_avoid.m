while(1)
    angle_speed = 0.3;
    wait_time = 3;
    [x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);
    dist1 = scan_data.Ranges(1:20);
    dist2 = scan_data.Ranges(340:360);
    dist1(dist1 == 0)=[];
    dist2(dist2 == 0)=[]; 
    minDist1 = min(dist1); 
    minDist2 = min(dist2);
    minDist = min(minDist1, minDist2);
    if minDist < 0.5
        disp("Avoid.......")
        if minDist1 < minDist2
            stop_mission(robot_pub);
            [velocity_msg]= generate_msgs(0.1, -angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < wait_time
%                 send_msgs(velocity_msg, robot_pub);
                disp("Turn right....");
            end
            [velocity_msg]= generate_msgs(0, angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < wait_time
%             send_msgs(velocity_msg, robot_pub);
            disp("Turn back ......");
            end
            stop_mission(robot_pub);
        else
            stop_mission(robot_pub);
            [velocity_msg]= generate_msgs(0.1, angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < wait_time
%                 send_msgs(velocity_msg, robot_pub);
                disp("Turn right....");
            end
            [velocity_msg]= generate_msgs(0, -angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < wait_time
%             send_msgs(velocity_msg, robot_pub);
            disp("Turn back ......");
            end
            stop_mission(robot_pub);
            states = 2;
        end
 else
        [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
        send_msgs(velocity_msg, robot_pub);
  end
end