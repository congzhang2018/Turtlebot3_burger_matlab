if ismember('/odom', rostopic('list'))
    odom_sub = rossubscriber('/odom');
end
turtelbot3 = receive(odom_sub);

% plot laser data
tic;
i = 0;
yaw = [];
  while toc < 1000
      
      turtelbot3 = receive(odom_sub,0.5);
%       robot_Orientation = [turtelbot3.Pose.Pose.Orientation.W, turtelbot3.Pose.Pose.Orientation.X, turtelbot3.Pose.Pose.Orientation.Y, turtelbot3.Pose.Pose.Orientation.Z];
%       robot_Rotation = quat2eul(robot_Orientation);
%       yaw = rad2deg(robot_Rotation(1));
      hold on
%       plot(i, yaw,'*');
%       i = i+1;
      %Plot the yaw angle
%       len_list = length(yaw);
%       i = [0:len_list];
%       plot(i, yaw,'*');
%       i = i + 1;
      % Plot the position of tb3
      plot(turtelbot3.Pose.Pose.Position.X, turtelbot3.Pose.Pose.Position.Y, "o");

  end
