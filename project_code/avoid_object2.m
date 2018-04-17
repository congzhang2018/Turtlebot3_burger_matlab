function Flag = avoid_object2(laser_sub, robot_pub) 
    angle_speed = 0.2;
    wait_time = 6.5;
    [x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);
    dist1 = scan_data.Ranges(1:35);
    dist2 = scan_data.Ranges(325:360);
    dist1(dist1 == 0)=[];
    dist2(dist2 == 0)=[]; 
    minDist1 = min(dist1); 
    minDist2 = min(dist2);
    minDist = min(minDist1, minDist2);
    %%% check if the minDist reach the limitation
    if minDist < 0.5
        disp("Avoid.......")
        %%% decide which side the robot should turn, if minDist1 < minDist2
        %%% turn left, otherwise turn right, and go straight for 25cm, at
        %%% last turn back and return to control loop
        if minDist1 < minDist2
            stop_mission(robot_pub);
            [velocity_msg]= generate_msgs(0, -angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Turn right....");
            tic;
            while toc < wait_time
            end
            [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Turn right....");
            tic;
            while toc < 2.5      
            end
            [velocity_msg]= generate_msgs(0, angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Turn back ......");
            tic;
            while toc < wait_time
            end
            stop_mission(robot_pub);
            Flag = false;
        else
            stop_mission(robot_pub);
            [velocity_msg]= generate_msgs(0, angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Turn right....");
            tic;
            while toc < wait_time
            end
            [velocity_msg]= generate_msgs(0.1,0, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Turn right....");
            tic;
            while toc < 2.5      
            end
            [velocity_msg]= generate_msgs(0, -angle_speed, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Turn back ......");
            tic;
            while toc < wait_time
            end
            stop_mission(robot_pub);
            Flag = false;
        end
    else
        Flag = true;
    end
end