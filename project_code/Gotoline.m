function [finsih_go_to_line]= Gotoline(distance, robot_pub)   

    cost_time = distance/0.1; %calculate time
    [velocity_msg]= generate_msgs(0.1, 0, robot_pub);
    send_msgs(velocity_msg, robot_pub);
    tic;
    while toc < cost_time
        disp("in state 3: Go to target >>>>>>>");
    end
    stop_mission(robot_pub);
    finsih_go_to_line = Ture;
    
end
