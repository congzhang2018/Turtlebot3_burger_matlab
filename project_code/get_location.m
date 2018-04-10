function [turtelbot3, robot_Rotation] = get_location()
    if ismember('/odom', rostopic('list'))
        odom_sub = rossubscriber('/odom');
    end
    turtelbot3 = receive(odom_sub,0.5);
    robot_Orientation = [turtelbot3.Pose.Pose.Orientation.W, turtelbot3.Pose.Pose.Orientation.X, turtelbot3.Pose.Pose.Orientation.Y, turtelbot3.Pose.Pose.Orientation.Z];
    robot_Rotation = quat2eul(robot_Orientation); 
    disp("Got location !!!");
end