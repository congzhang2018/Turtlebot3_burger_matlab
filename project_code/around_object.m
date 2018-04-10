function around_object()
    [x_lidar, y_lidar, scan_data] = get_lidar_data();
    dist1 = scan_data.Ranges(1:20);
    dist2 = scan_data.Ranges(340:360);
    dist1(dist1 == 0)=[];
    dist2(dist2 == 0)=[]; 
    minDist1 = min(dist1); 
    minDist2 = min(dist2);
    minDist = min(minDist1, minDist2);
    if minDist < 0.5
        stop_mission(robot_pub);
        [robot_Rotation] = get_imu_data();
        current_yaw = robot_Rotation(1);
        record_yaw = current_yaw;
        desired_yaw = current_yaw + 1.57;
        if desired_yaw > 3.14
            desired_yaw = desired_yaw - 2*pi;
        elseif desired_yaw < -3.14
            desired_yaw = desired_yaw + 2*pi ;
        end
        while abs(abs(desired_yaw) - abs(current_yaw)) > 0.052   
            [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
            send_msgs(velocity_msg); 
            disp("send msgs!!");
            disp(desired_yaw);
            disp(current_yaw);
            [robot_Rotation] = get_imu_data();
            current_yaw = robot_Rotation(1);
        end  
        stop_mission(robot_pub);
        [velocity_msg] = generate_msgs(0.1,0,robot_pub);
        send_msgs(velocity_msg);
        pause(2);

        stop_mission(robot_pub);
        [robot_Rotation] = get_imu_data();
        current_yaw = robot_Rotation(1);
        desired_yaw = current_yaw - 1.57;
        if desired_yaw > 3.14
            desired_yaw = desired_yaw - 2*pi;
        elseif desired_yaw < -3.14
            desired_yaw = desired_yaw + 2*pi;
        end
        while abs(abs(desired_yaw) - abs(current_yaw)) > 0.052 
            [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
            send_msgs(velocity_msg); 
            disp("send msgs!!");
            disp(desired_yaw);
            disp(current_yaw);
            [robot_Rotation] = get_imu_data();
            current_yaw = robot_Rotation(1);
        end  
%         [velocity_msg] = generate_msgs(0.1,0,robot_pub);
%         send_msgs(velocity_msg);
%         pause(2);
%         stop_mission(robot_pub);
%         
%         [robot_Rotation] = get_imu_data();
%         current_yaw = robot_Rotation(1);
%         desired_yaw = current_yaw - 1.57;
%         if desired_yaw > 3.14
%             desired_yaw = desired_yaw - 2*pi;
%         elseif desired_yaw < -3.14
%             desired_yaw = desired_yaw + 2*pi;
%         end
%         while abs(abs(desired_yaw) - abs(current_yaw)) > 0.052 
%             [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
%             send_msgs(velocity_msg); 
%             disp("send msgs!!");
%             disp(desired_yaw);
%             disp(current_yaw);
%             [robot_Rotation] = get_imu_data();
%             current_yaw = robot_Rotation(1);
%         end  
%         [velocity_msg] = generate_msgs(0.1,0,robot_pub);
%         send_msgs(velocity_msg);
%         pause(2);
%         stop_mission(robot_pub);
%         
%          desired_yaw = record_yaw;
%         if desired_yaw > 3.14
%             desired_yaw = desired_yaw - 2*pi;
%         elseif desired_yaw < -3.14
%             desired_yaw = desired_yaw + 2*pi ;
%         end
%         while abs(desired_yaw - current_yaw) > 0.052   
%             [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
%             send_msgs(velocity_msg); 
%             disp("send msgs!!");
%             disp(desired_yaw);
%             disp(current_yaw);
%             [robot_Rotation] = get_imu_data();
%             current_yaw = robot_Rotation(1);
%         end  
%         stop_mission(robot_pub);
    else
        [velocity_msg] = generate_msgs(0.1,0,robot_pub);
        send_msgs(velocity_msg);
        disp("go forword!!!")
    end
end