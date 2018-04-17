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
finsih_go_to_line = true;
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
       
       [velocity_msg, minDist] = aviod_object(scan_data, robot_pub);
       
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
        [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
        send_msgs(velocity_msg, robot_pub);
        tic;
        while toc < 1
        end
        Flag = around_object2(laser_sub, robot_pub);
        if Flag 
%                             disp("no object in the forward");
        else
            states = 2;
        end
        disp("End aviod!!");
        stop_mission(robot_pub);
        [robot_Rotation] = get_imu_data(imu_sub);
        disp("Get imu data");
        [distance_pillar, angle_pillar, distance_line, angle_line, angle]= get_cam_data(cam_sub);
        disp("Get cam data");
        
        if angle_pillar == -1
            states = 2;
            disp("not find yellow polar!!");            
        else
            record_yellow_angle = robot_Rotation(1) + angle_pillar;
            %%% Turn the heading face to the white line
            
            if angle_pillar > 0
                [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
            else 
                [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
            end
            send_msgs(velocity_msg, robot_pub);
            disp("Turning to face the pillar......");
            tic;
            while toc < 2        
            end
            stop_mission(robot_pub);
            disp("Face to the pillar!!");
            
            states = 4;
            
        end
       
    elseif states == 3
            disp("In state 3!!!!");
            [robot_Rotation] = get_imu_data(imu_sub);
            [distance_pillar, angle_pillar, distance_line, angle_line, angle]= get_cam_data(cam_sub);
            disp("Get cam data");
            if distance_line == -1
                states = 3;
                [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
                send_msgs(velocity_msg, robot_pub);
%                 stop_mission(robot_pub);
                disp("in state 3 :lost line return to searching....")
            else
                if finsih_go_to_line
                    finsih_go_to_line = Gotoline(distance_line,angle, robot_pub); 
                    disp("in state 3:Go to white line ... ");
                else
                    cost_time = 1;
                    [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
                    send_msgs(velocity_msg, robot_pub);
                    disp("in state 3:Follow white line ... ");
                    tic;
                    while toc < cost_time
                    end
                    stop_mission(robot_pub);
                    Flag = around_object3(laser_sub, robot_pub);
                    if Flag 
%                             disp("no object in the forward");
                    else
                        states = 2;
                    end
                end
                [robot_Rotation] = get_imu_data(imu_sub);
                current_yaw = robot_Rotation(1);
                if angle_line > 0
                    [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                end
                send_msgs(velocity_msg, robot_pub);
                disp("in state 3: Turning to face the polar......");
                tic;
                while toc < 1
                end 
                
                stop_mission(robot_pub);
                Flag = around_object2(laser_sub, robot_pub);
                if Flag 
%                             disp("no object in the forward");
                else
                    states = 2;
                end
                disp(" in stat 3:te Face to the polar");
                states = 4;

            end
       
    elseif states == 4
        disp("In state 4!!!!");
        [distance_pillar, angle_pillar, distance_line, angle_line, angle]= get_cam_data(cam_sub);
        disp("in stat 4:Get cam data");
        if angle_line == -1
            if distance_pillar == -1
                disp("in state 4 :lost target... return to searching ....");
                states = 2;
            else
                [x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);       
                [velocity_msg, minDist] = aviod_object(scan_data, robot_pub);
                
                if (minDist < 0.7) && (distance_pillar < 3)
                    stop_mission(robot_pub);
                    disp(minDist);
                    disp("Arrival position!!");
                    break;
                else
                    cost_time = 3;
                    [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
                    send_msgs(velocity_msg, robot_pub);
                    disp("in state 4:Moving to the yellow polar ... ");
                    tic;
                    while toc < cost_time
                    end
%                     stop_mission(robot_pub);
                    Flag = around_object2(laser_sub, robot_pub);
                    if Flag 
%                             disp("no object in the forward");
                    else
                        states = 2;
                    end
                    [distance_pillar, angle_pillar, distance_line, angle_line, angle]= get_cam_data(cam_sub);
                    disp(angle_pillar);
                    cost_angle_time = angle_pillar/0.3;
                    if angle_pillar > 0
                        [velocity_msg] = generate_msgs(0, -0.2, robot_pub);
                    else 
                        [velocity_msg] = generate_msgs(0, 0.2, robot_pub);
                    end
                    send_msgs(velocity_msg, robot_pub);
                    disp("in state 4: Turning to face the polar......");
                    tic;
                    while toc < cost_angle_time             
                    end
                    stop_mission(robot_pub);
                    Flag = around_object2(laser_sub, robot_pub);
                    if Flag 
%                             disp("no object in the forward");
                    else
                        states = 2;
                    end
                end
            end
        else
            cost_time1 = 2;
            [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            disp("Following line ... ");
            tic;
            while toc < cost_time1
            end
            stop_mission(robot_pub);
            Flag = around_object3(laser_sub, robot_pub);
            if Flag 
%                             disp("no object in the forward");
            else
                states = 2;
            end
            if abs(angle_line) < 0.1
                [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
            else
                if angle_line > 0
                    [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                end
            end
            send_msgs(velocity_msg, robot_pub);
            disp("in state 4: Turning to face the polar......");
            tic;
            while toc < 1
            end 
        end      
        stop_mission(robot_pub);
        disp('Stop tutrlebot!!');
     end
end

   











