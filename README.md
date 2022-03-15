# ROS_Network_Info

may also be easiler to use **sudo nano /etc/hosts** to set the ip addresses a name

YAAYY I fianlly managed to do a hello world accross multiple machines and this is the codes needed to do so

M#  = Machine (number)

Steps

-M1: roscore

-M2: export ROS_MASTER_URI=http://ip_address:11311/

-M2: nano /etc/host

-M2: add the ip address of the master machine under the other addressses so it will read for example

-127.0.0.1       localhost

-127.0.1.1       Blackbox

-172.30.58.194   Rover1

-M1&M2 :  make sure the devel/setup.bash is sourced

-M1&M2 : rosrun
