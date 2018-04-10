if ismember('/imu', rostopic('list'))
    imu_sub = rossubscriber('/imu');
end
turtelbot3 = receive(imu_sub);

% plot laser data
tic;
i = 0;
yaw = [];
  while toc < 1000
      
      turtelbot3 = receive(imu_sub,0.5);
      robot_Orientation = [turtelbot3.Orientation.W, turtelbot3.Orientation.X, turtelbot3.Orientation.Y, turtelbot3.Orientation.Z];
      robot_Rotation = quat2eul(robot_Orientation);
%       yaw = rad2deg(robot_Rotation(1));
      hold on
%       Plot the yaw angle
%       len_list = length(yaw);
%       i = [0:len_list];
      plot(i, robot_Rotation(1),'*');
      i = i+1;
      % Plot the position of tb3
%       plot3(turtelbot3.Pose.Pose.Position.X, turtelbot3.Pose.Pose.Position.Y, turtelbot3.Pose.Pose.Position.Z, "o");

  end
