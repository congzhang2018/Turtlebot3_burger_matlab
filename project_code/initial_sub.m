function [imu_sub, cam_sub, laser_sub] = initial_sub()
    laser_sub = rossubscriber('/scan');
    imu_sub = rossubscriber('/imu');
    cam_sub = rossubscriber('/telemetry');
end