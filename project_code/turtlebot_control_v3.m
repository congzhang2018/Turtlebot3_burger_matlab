clear all; close all; clc;
%% Connect to Turtlebot
% Connect to an External ROS Master
% ip_robot = '192.168.1.101';   % ip address of robot, replace this one with yours
% rosinit(ip_robot, 'NodeHost','192.168.1.103');
% robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
%% state
states = 1;  % initial state
% states = 2;  % searching state
% states = 3;  % Go to line state
% states = 4;  % Follow line and go to pillar state
robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
laser_sub = rossubscriber('/scan');
imu_sub = rossubscriber('/imu');
cam_sub = rossubscriber('/telemetry');
finsih_go_to_line = false;
% pi_cam_node = '/raspicam_node/image/compressed';
% image_sub = rossubscriber(pi_cam_node);
% [imu_sub, cam_sub, laser_sub] = initial_sub();
%% Control loop
while(1)
    disp("Loop stat !!!!");
    if states == 1
       disp("In state 1!!!!");
       [robot_Rotation] = get_imu_data(imu_sub);
       current_yaw = robot_Rotation(1);
       [x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);
       
       [velocity_msg, minDist] = aviod_object(x_lidar, y_lidar, robot_pub);
       
       if minDist < 0.5
            [msg] = generate_msgs(-0.1, 0, robot_pub);
            send_msgs(velocity_msg, robot_pub); 
       end
       
       stop_mission(robot_pub);
       states = 3;
       disp('Change to searching state!!');
       
    elseif states == 2
        disp("In state 2!!!!");
       %%% Searching for white line and yellow point   
        [velocity_msg] = generate_msgs(0, 0.5, robot_pub);
        send_msgs(velocity_msg, robot_pub);
        tic;
        while toc < 2
            disp("Turning 60 degrees......");
        end
        disp("End Turning!!");
        stop_mission(robot_pub);
        [robot_Rotation] = get_imu_data(imu_sub);
        disp("Get imu data");
        [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
        disp("Get cam data");
        
        if angle_polar == -1
            states = 2;
            pause(5);
            disp("not find yellow polar!!");
        else
            record_yellow_angle = robot_Rotation(1) + angle_polar;
            %%% Turn the heading face to the white line
            
            if angle_polar > 0
                [velocity_msg] = generate_msgs(0, -0.5, robot_pub);
            else 
                [velocity_msg] = generate_msgs(0, 0.5, robot_pub);
            end
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 2    
                disp("Turning to face the pillar......");
            end
            stop_mission(robot_pub);
            disp("Face to the pillar!!");
            
            states = 4;
            
        end
       
    elseif states == 3
            disp("In state 3!!!!");
            [robot_Rotation] = get_imu_data(imu_sub);
            [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
            disp("Get cam data");
            if distance_line == -1
                states = 3;
                disp("in state 3 :lost line return to searching....")
            else
                if finsih_go_to_line
                    finsih_go_to_line = Gotoline(distance_line, robot_pub); 
                    disp("in state 3:Go to white line ... ");
                else
                    cost_time = 2;
                    [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
                    send_msgs(velocity_msg, robot_pub);
                    tic;
                    while toc < cost_time
                    disp("in state 3:Follow white line ... ");
                    end
                    stop_mission(robot_pub);
                end

                [robot_Rotation] = get_imu_data(imu_sub);
                current_yaw = robot_Rotation(1);
                desired_yaw = current_yaw - angle_line;
%                 desired_yaw = angle_polar;
                disp(desired_yaw);
                if desired_yaw > 0
                    [velocity_msg] = generate_msgs(0, 0.3, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, -0.3, robot_pub);
                end
                send_msgs(velocity_msg, robot_pub);
                tic;
                while toc < 2    
                    disp("in state 3: Turning to face the polar......");
                end 
                stop_mission(robot_pub);
                disp(" in stat 3:te Face to the polar");
                states = 4;
            end
       
    elseif states == 4
        disp("In state 4!!!!");
        [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
        disp("in stat 4:Get cam data");
        if distance_line == -1
            if distance_polar == -1
                disp("in state 4 :lost target... return to searching ....");
                states = 2;
            else
                [x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);       
                [velocity_msg, minDist] = aviod_object(x_lidar, y_lidar, robot_pub);
                if minDist < 0.5
                    stop_mission(robot_pub);
                    disp("Arrival position!!");
                else
                    cost_time = 5;
                    [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
                    send_msgs(velocity_msg, robot_pub);
                    tic;
                    while toc < cost_time
                        disp("in state 4:Moving to the yellow polar ... ");
                    end
                    stop_mission(robot_pub);
                    [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
                    disp(angle_polar);
                    cost_angle_time = angle_polar/0.5;
                    if angle_polar > 0
                        [velocity_msg] = generate_msgs(0, -0.2, robot_pub);
                    else 
                        [velocity_msg] = generate_msgs(0, 0.2, robot_pub);
                    end
                    disp(angle_time);
                    send_msgs(velocity_msg, robot_pub);
                    
                    tic;
                    while toc < angle_time
                        disp("in state 4: Turning to face the polar......");
                    end
                    stop_mission(robot_pub);
                end
            end
        else
            cost_time = 2;
            [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < cost_time
                disp("Moving to the yellow polar ... ");
            end
            stop_mission(robot_pub);
        end      
        stop_mission(robot_pub);
        disp('Stop tutrlebot!!');
     end
end

   











