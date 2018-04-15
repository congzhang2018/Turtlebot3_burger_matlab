function [velocity_msg, minDist] = aviod_object(scan_data, robot_pub)
    
    dist1 = scan_data.Ranges(1:30);
    dist2 = scan_data.Ranges(330:360);
    dist1(dist1 == 0)=[];
    dist2(dist2 == 0)=[]; 
    minDist1 = min(dist1); 
    minDist2 = min(dist2);
    minDist = min(minDist1, minDist2);
    
    velocity_msg = rosmessage(robot_pub);
    % Command robot action
    velocity_msg.Angular.Z = 0;
    velocity_msg.Linear.X = 0;
end