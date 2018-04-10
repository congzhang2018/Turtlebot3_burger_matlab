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
%% Control loop
while(1)

    if states == 1
       [robot_Rotation] = get_imu_data();
       current_yaw = robot_Rotation(1);
%        current_location = turtelbot3.Pose.Pose.Position;
       [x_lidar, y_lidar] = get_lidar_data();
       [velocity_msg, minDist] = aviod_object(x_lidar, y_lidar, robot_pub);
       if minDist < 0.5
            send_msgs(velocity_msg); 
       end
       states = 2;
       disp('Change to searching state!!');
       pause(5);
       
    elseif states == 2
       stop_mission(robot_pub);
       %%% Searching for white line and yellow point
       for i = 1:6
            %%% Turn the heading 60 degrees
            [robot_Rotation] = get_imu_data();
            current_yaw = robot_Rotation(1);
            desired_yaw = current_yaw + 1.047;
            if desired_yaw > 3.14
                desired_yaw = desired_yaw - 2*pi;
            elseif desired_yaw < -3.14
                desired_yaw = desired_yaw + 2*pi ;
            end      
            while abs(desired_yaw - current_yaw) > 0.3
                [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                send_msgs(velocity_msg); 
                disp("send msgs!!");
                disp(desired_yaw);
                disp(current_yaw);
                disp("Turning angle...")
                [robot_Rotation] = get_imu_data();
                current_yaw = robot_Rotation(1);
            end  
            stop_mission(robot_pub);
            %%% Check the camera data, if found turn heading face to line
            %%% return states = 3, else continue searching.
            [robot_Rotation] = get_imu_data();
            [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data();
            if distance_line == -1
                disp("arrival points!!");
                pause(5);
            else
                %%% Turn the heading face to the white line
                current_yaw = robot_Rotation(1);
                desired_yaw = current_yaw + angle_line;
                if distance_polar == -1
                    Found_polar = false;
                else
                    record_polar_yaw = current_yaw + angle_polar;
                end
                if desired_yaw > 3.14
                    desired_yaw = desired_yaw - 2*pi;
                elseif desired_yaw < -3.14
                    desired_yaw = desired_yaw + 2*pi ;
                end      
                while abs(desired_yaw - current_yaw) > 0.3
                    [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                    send_msgs(velocity_msg); 
                    disp("send msgs!!");
                    disp(desired_yaw);
                    disp(current_yaw);
                    disp("facing... to line")
                    [robot_Rotation] = get_imu_data();
                    current_yaw = robot_Rotation(1);
                end  
                stop_mission(robot_pub);
                states = 3;
                disp("Face to the line!!");
                pause(10);
                break;
            end
       end
       
    elseif states == 3
            [robot_Rotation] = get_imu_data();
            [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data();
            if distance_line == -1
                states = 2;
                return
            else
                cost_time = distance_line/0.1; %calculate time 
                current_yaw = robot_Rotation(1);
                desired_yaw = angle_line + current_yaw; 

                if desired_yaw > 3.14
                    desired_yaw = desired_yaw - 2*pi;
                elseif desired_yaw < -3.14
                    desired_yaw = desired_yaw + 2*pi ;
                end   

                while abs(desired_yaw - current_yaw) > 0.174
                    [velocity_msg] = generate_msgs(0, 0.1, robot_pub);
                    send_msgs(velocity_msg); 
                    disp("send msgs!!");
                    disp(desired_yaw);
                    disp(current_yaw);
                    [robot_Rotation] = get_imu_data();
                    current_yaw = robot_Rotation(1);
                end  
                stop_mission(robot_pub);
                [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
                tic;
                while toc < cost_time
                    [x_lidar, y_lidar, scan_data] = get_lidar_data();
                    [msg, minDist] = aviod_object(x_lidar, y_lidar, robot_pub);
                    if minDist > 0.5
    %                     around_object();
    %                     around_object2();
    %                     stop_mission();
                        send_msgs(msg);
                    else
                        send_msgs(velocity_msg);
                    end
                end
                stop_mission(robot_pub);
            end
    else
        [velocity_msg] = stop_mission(robot_pub);
        disp('Stop tutrlebot!!');
    end
end











