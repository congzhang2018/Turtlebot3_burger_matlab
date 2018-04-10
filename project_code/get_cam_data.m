function [distance_polar, angle_polar, distance_line, angle_line]= get_cam_data(cam_sub)
% if ismember('/telemetry', rostopic('list'))
%     cam_sub = rossubscriber('/telemetry');
% end
tic;
cam_data = receive(cam_sub);
distance_polar = cam_data.Linear.X;
angle_polar = cam_data.Linear.Y;
distance_line = cam_data.Angular.X; 
angle_line = cam_data.Angular.Y;
toc;
end