function [robot_Rotation] = get_imu_data(imu_sub)

imu_data = receive(imu_sub);
robot_Orientation = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
robot_Rotation = quat2eul(robot_Orientation);

end