
msg= generate_msgs(0.1, 0, robot_pub);
% send_msgs(msg, robot_pub);
for i = 1:15
send_msgs(msg, robot_pub);
% tic;
%                                                                                                                                     
pause(2);
% toc;
stop_mission(robot_pub);
end