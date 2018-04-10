function [velocity_msg]= generate_msgs(vu, yaw, robot_pub)

    velocity_msg = rosmessage(robot_pub);
    % assign linear velocity
    velocity_msg.Linear.X = vu;
    velocity_msg.Linear.Y = 0;
    velocity_msg.Linear.Z = 0;
    % assign angular velocity
    velocity_msg.Angular.X = 0;
    velocity_msg.Angular.Y = 0;
    velocity_msg.Angular.Z = yaw;
    
end