function send_msgs(velocity_msg)
    robot_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
    send(robot_pub,velocity_msg);
end