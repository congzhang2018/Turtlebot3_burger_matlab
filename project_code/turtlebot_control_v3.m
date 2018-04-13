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
            send_msgs(velocity_msg, robot_pub); 
       end
       
       stop_mission(robot_pub);
       states = 2;
       disp('Change to searching state!!');
       
    elseif states == 2
        disp("In state 2!!!!");
       %%% Searching for white line and yellow point   
        [velocity_msg] = generate_msgs(0, 0.5, robot_pub);
        send_msgs(velocity_msg, robot_pub);
        tic;
        while toc < 2.2
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
            disp("not find yellow polar!!");
        else
            record_yellow_angle = robot_Rotation(1) + angle_polar;
            if distance_line == -1
                states = 2;
                disp("not find line!!");
            else
                %%% Turn the heading face to the white line
                angle_time = abs(angle_line-1.57)/0.5;
                if angle_line > 0
                    [velocity_msg] = generate_msgs(0, -0.5, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, 0.5, robot_pub);
                end
                send_msgs(velocity_msg, robot_pub);
                tic;
                while toc < angle_line    
                    disp("Turning to face the line......");
                end
                stop_mission(robot_pub);
                disp("Face to the line!!");
                states = 3;
            end
        end
       
    elseif states == 3
            disp("In state 3!!!!");
            [robot_Rotation] = get_imu_data(imu_sub);
            [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
            disp("Get cam data");
            if distance_line == -1
                states = 2;
                disp("lost line return to searching....")
            else
                cost_time = distance_line/0.5; %calculate time
                [velocity_msg]= generate_msgs(0.5, 0, robot_pub);
                send_msgs(velocity_msg, robot_pub);
                tic;
                while toc < cost_time
                    disp("Go to target >>>>>>>");
                end
                stop_mission(robot_pub);
                [robot_Rotation] = get_imu_data(imu_sub);
                current_yaw = robot_Rotation(1);
                desired_yaw = record_yellow_angle - current_yaw;
                angle_time = abs(desired_yaw)/0.5;
                if desired_yaw > 0
                    [velocity_msg] = generate_msgs(0, 0.5, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, -0.5, robot_pub);
                end
                send_msgs(velocity_msg, robot_pub);
                tic;
                while toc < angle_time    
                    disp("Turning to face the polar......");
                end
                
                stop_mission(robot_pub);
                disp("Face to the polar");
                states = 4;
            end
       
    elseif states == 4
        disp("In state 4!!!!");
        [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
        disp("Get cam data");
        if distance_line == -1
            if distance_polar == -1
                disp("lost target... return to searching ....");
                states = 2;
            else
                if distance_polar < 0.5
                    stop_mission(robot_pub);
                    disp("Arrival position!!");
                else
                    cost_time = distance_polar/0.5;
                    [velocity_msg] = generate_msgs(0.5, 0, robot_pub);
                    send_msgs(velocity_msg, robot_pub);
                    tic;
                    while toc < cost_time
                        diso("Moving to the yellow polar ... ");
                    end
                    stop_mission(robot_pub);
                    cost_angle_time = angle_polar/0.5;
                    if desired_yaw > 0
                        [velocity_msg] = generate_msgs(0, 0.5, robot_pub);
                    else 
                        [velocity_msg] = generate_msgs(0, -0.5, robot_pub);
                    end
                    send_msgs(velocity_msg, robot_pub);
                    tic;
                    while toc < angle_time
                        disp("Turning to face the polar......");
                    end
                    stop_mission(robot_pub);
                end
            end
        else
            cost_time = distance_line/0.5;
            [velocity_msg] = generate_msgs(0.5, 0, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < cost_time
                diso("Moving to the yellow polar ... ");
            end
            stop_mission(robot_pub);
%             cost_angle_time = angle_line/0.1;
%             if desired_yaw > 0
%                 [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
%             else 
%                 [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
%             end
%             tic;
%             while toc < angle_time
%                 send_msgs(velocity_msg, robot_pub);
%                 disp("Turning to face the polar......");
%             end
            stop_mission(robot_pub);
        end      
        [velocity_msg] = stop_mission(robot_pub);
        disp('Stop tutrlebot!!');
     end
end

   











