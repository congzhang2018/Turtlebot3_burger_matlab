function  connect_turtlebot3()

ip_robot = '192.168.1.101';   % ip address of robot, replace this one with yours
rosinit(ip_robot, 'NodeHost','192.168.1.103');

end