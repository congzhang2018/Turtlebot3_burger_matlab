clear all; close all; clc;
%% Connect to Turtlebot
% Connect to an External ROS Master
% ip_robot = '192.168.1.101';   % ip address of robot, replace this one with yours
% rosinit(ip_robot, 'NodeHost','192.168.1.103');
% robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
%% state
states = 1;  % initial state
% states = 2;  % searching state
% states = 3;  % line following state
% states = 4;  % Finsih task, stop
robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
[imu_sub, cam_sub, laser_sub] = initial_sub();
%% Control loop
while(1)

    if states == 1
       [robot_Rotation] = get_imu_data(imu_sub);
       current_yaw = robot_Rotation(1);
%        current_location = turtelbot3.Pose.Pose.Position;
       [x_lidar, y_lidar] = get_lidar_data();
       [velocity_msg, minDist] = aviod_object(x_lidar, y_lidar, robot_pub);
       if minDist < 0.5
            send_msgs(velocity_msg, robot_pub); 
       end
       states = 2;
       disp('Change to searching state!!');
       stop_mission(robot_pub);
       pause(5);
       
    elseif states == 2
       %%% Searching for white line and yellow point   
        [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
        tic;
        while toc < 10.47
            send_msgs(velocity_msg, robot_pub);
            disp("Turning 60 degrees......");
        end
        stop_mission(robot_pub);
        [robot_Rotation] = get_imu_data(imu_sub);
        [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
        
        if angle_polar == -1
            states = 2;
            disp("not find yellow polar!!");
            return
        else
            record_yellow_angle = robot_Rotation(1) + angle_polar;
            if distance_line == -1
                states = 2;
                disp("not find line!!");
                return
            else
                %%% Turn the heading face to the white line
                angle_time = abs(angle_line)/0.1;
                if angle_line > 0
                    [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
                end
                tic;
                while toc < angle_line
                    send_msgs(velocity_msg, robot_pub);
                    disp("Turning to face the line......");
                end
                stop_mission(robot_pub);
                disp("Face to the line!!");
                states = 3;
                break;
            end
        end
       
    elseif states == 3
            [robot_Rotation] = get_imu_data(imu_sub);
            [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
            if distance_line == -1
                states = 2;
                disp("lost line return to searching....")
                return
            else
                cost_time = distance_line/0.1; %calculate time
                [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
                tic;
                while toc < cost_time
                    send_msgs(velocity_msg, robot_pub);
                end
                stop_mission(robot_pub);
                [robot_Rotation] = get_imu_data(imu_sub);
                current_yaw = robot_Rotation(1);
                desired_yaw = record_yellow_angle - current_yaw;
                angle_time = abs(desired_yaw)/0.1;
                if desired_yaw > 0
                    [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                else 
                    [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
                end
                tic;
                while toc < angle_time
                    send_msgs(velocity_msg, robot_pub);
                    disp("Turning to face the polar......");
                end
                stop_mission(robot_pub);
                disp("Face to the polar");
                states = 4;
            end
    elseif states == 4
        
        [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub);
        
        if distance_line == -1
            if distance_polar == -1
                disp("lost target... return to searching ....");
                states = 2;
                return
            else
                if distance_polar < 0.5
                    stop_mission(robot_pub);
                    disp("Arrival position!!");
                    return
                else
                    cost_time = distance_polar/0.1;
                    [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
                    tic;
                    while toc < cost_time
                        send_msgs(velocity_msg, robot_pub);
                        diso("Moving to the yellow polar ... ");
                    end
                    stop_mission(robot_pub);
                    cost_angle_time = angle_polar/0.1;
                    if desired_yaw > 0
                        [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                    else 
                        [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
                    end
                    tic;
                    while toc < angle_time
                        send_msgs(velocity_msg, robot_pub);
                        disp("Turning to face the polar......");
                    end
                    stop_mission(robot_pub);
                end
            end
        else
            cost_time = distance_line/0.1;
            [velocity_msg] = generate_msgs(0.1, 0, robot_pub);
            tic;
            while toc < cost_time
                send_msgs(velocity_msg, robot_pub);
                diso("Moving to the yellow polar ... ");
            end
            stop_mission(robot_pub);
            cost_angle_time = angle_line/0.1;
            if desired_yaw > 0
                [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
            else 
                [velocity_msg] = generate_msgs(0, -0.1, robot_pub);
            end
            tic;
            while toc < angle_time
                send_msgs(velocity_msg, robot_pub);
                disp("Turning to face the polar......");
            end
            stop_mission(robot_pub);
        end      
        [velocity_msg] = stop_mission(robot_pub);
        disp('Stop tutrlebot!!');
    end
end











