function [finsih_go_to_line]= Gotoline(distance,angle, robot_pub)   

    cost_time = (distance - 0.07)/0.1; %calculate time
    [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
    send_msgs(velocity_msg, robot_pub);
    disp("in Go to line: Go to target >>>>>>>");
    tic;
    while toc < cost_time   
    end
    stop_mission(robot_pub);
    time_cost = abs(angle)/0.3;
    if angle > 0
        [velocity_msg]= generate_msgs(0, -0.2, robot_pub);
    else
        [velocity_msg]= generate_msgs(0, 0.2, robot_pub);
    end
    send_msgs(velocity_msg, robot_pub);
    disp("in Go to line: turning to face the pillar");
    tic;
    while toc < time_cost
    end
    stop_mission(robot_pub);
    finsih_go_to_line = false;
end
