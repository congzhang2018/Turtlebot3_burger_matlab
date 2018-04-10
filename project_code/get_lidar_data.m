function [x_lidar, y_lidar, scan_data] = get_lidar_data() 
    if ismember('/scan', rostopic('list'))
        laser_sub = rossubscriber('/scan');
    end
    scan_data = receive(laser_sub);
    data = readCartesian(scan_data);
    x_lidar = data(:,1);
    y_lidar = data(:,2);  
end