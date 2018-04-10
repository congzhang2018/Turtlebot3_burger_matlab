function around_object2()

[x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);
dist1 = scan_data.Ranges(1:20);
dist2 = scan_data.Ranges(340:360);
dist1(dist1 == 0)=[];
dist2(dist2 == 0)=[]; 
minDist1 = min(dist1); 
minDist2 = min(dist2);
minDist = min(minDist1, minDist2);
if minDist < 0.5
    if minDist1 < minDist2
        stop_mission(robot_pub);
        [velocity_msg]= generate_msgs(0, 0.1, robot_pub);
        tic;
        while toc < 15.7
            send_msgs(velocity_msg);
        end
        stop_mission(robot_pub);
        [velocity_msg]= generate_msgs(0, -0.1, robot_pub);

        tic;
        while toc < 17
            send_msgs(velocity_msg);
        end
        stop_mission(robot_pub);
    else
         stop_mission(robot_pub);
        [velocity_msg]= generate_msgs(0, -0.1, robot_pub);
        tic;
        while toc < 15.7
            send_msgs(velocity_msg);
        end
        stop_mission(robot_pub);
        [velocity_msg]= generate_msgs(0, 0.1, robot_pub);

        tic;
        while toc < 17
            send_msgs(velocity_msg);
        end
        stop_mission(robot_pub); 
    end

end