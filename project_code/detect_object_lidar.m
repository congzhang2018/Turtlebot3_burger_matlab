function [minDist] = detect_object_lidar(laser_sub)
    [x_lidar, y_lidar, scan_data] = get_lidar_data(laser_sub);
    dist1 = scan_data.Ranges(1:20);
    dist2 = scan_data.Ranges(340:360);
    dist1(dist1 == 0)=[];
    dist2(dist2 == 0)=[]; 
    minDist1 = min(dist1); 
    minDist2 = min(dist2);
    minDist = min(minDist1, minDist2);
end