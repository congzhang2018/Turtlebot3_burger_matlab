function [distance_polar, angle_polar, distance_line, angle_line, angle]= get_cam_data(cam_sub)

cam_data = receive(cam_sub);
distance_polar = cam_data.Linear.X;
angle_polar = cam_data.Linear.Y;
distance_line = cam_data.Angular.X; 
angle_line = cam_data.Angular.Y;
angle = cam_data.Angular.Z;

end