% function around_object2()
clear all; clc;
laser_sub = rossubscriber('/scan');
robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');

while(1)
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
            [velocity_msg]= generate_msgs(0, 0.2, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 6.5
%                 send_msgs(velocity_msg, robot_pub);
                disp("Turn right....");
            end
            stop_mission(robot_pub);
            
            disp("wait for 5 seconds");
%             pause(5);
            
            [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 4
%             send_msgs(velocity_msg, robot_pub);
            disp("go forward ......");
            end
            
            stop_mission(robot_pub);
            disp("wait for 5 seconds");
%             pause(5);
            
            [velocity_msg]= generate_msgs(0, -0.2, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 6.5
%                 send_msgs(velocity_msg, robot_pub);
                disp("Turn left....");
            end
            stop_mission(robot_pub);
            
            disp("wait for 5 seconds");
%             pause(5);
            
        else
             stop_mission(robot_pub);
            [velocity_msg]= generate_msgs(0, -0.2, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 6.5
%                 send_msgs(velocity_msg, robot_pub);
                disp("Turn left....");
            end
            stop_mission(robot_pub);
            
            disp("wait for 5 seconds");
%             pause(5);
            
            [velocity_msg]= generate_msgs(0.1, 0, robot_pub);   
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 4
%             send_msgs(velocity_msg, robot_pub);
            disp("go forward ......");
            end
            stop_mission(robot_pub);
            disp("wait for 5 seconds");
%             pause(5);
            
            [velocity_msg]= generate_msgs(0, 0.2, robot_pub);
            send_msgs(velocity_msg, robot_pub);
            tic;
            while toc < 6.5
%                 send_msgs(velocity_msg, robot_pub);
                disp("Turn right....");
            end
            stop_mission(robot_pub); 
            disp("wait for 5 seconds");
%             pause(5);
        end
    else
       [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
       send_msgs(velocity_msg, robot_pub); 
       disp("Go forword!!!")
    end
end