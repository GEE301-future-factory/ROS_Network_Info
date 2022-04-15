# ROS_Network_Info

### HAVENT TESTED IF COMMUNNICATION IS BI-DIRECTIONAL YET. ALL NODES ARE REJESTERD WITH THE MASTER BUT DATA MAY NOT BE EXCHANGED ### 
This is how I have currently left the setup for the ROS network.
The whole system will be able to be booted by the comands 

cd gee301_ws/

source devel/setup.bash

roslaunch system system_start.launch

Running Nodes

Locally
 - Uarm_Control_1
 - rovot_state_publisher
 - swiftpro_rvis_node

Rover_1 PI
 - Rover1_Arduino_send
 - RFID_Node
 - Docking   #This node kept crashing when trying to launch it and so i dont know if this need looking at or its a lunch problem
 - swiftpro_write_node

Dispenser PI
 - Dispenser_1
 - Dispenser_2
 - Dispenser_3

Blackbox PI
 - BlackBox_1
 - BlackBox_2
 - BlackBox_3
 - Handover

GlueGun PI
 - GlueGlueControler  # this is spelt wrong in the file name so i just used the wrong spelling rather than correcting

### THIS HAS NOT BEEN COMPLEATLY TESTED ###
  
I didnt have time to compleatly test this launch file before I had to rush off. The change I made is to move the Uarm to be able to be plugged into the Rover_1 PI rather than running locally. I think I identified which node is needed to run to connect to the uarm but it is just a educated guess. A similar thing will be needed to be done for the mover6 rqt_graph is especially helpfull with this.

### How to add more nodes to the launch file ###

The launch file is written html. and is broken down into a group for each raspberry pi. This is not nessassaary but done to keep things tidy. The format is:

<node machine="NAME OF MACHINE" name="NAME OF NODE THAT YOU WANT TO CALL IT" pkg="NAME OF PACKAGE - (the first folder inside the src folder)" type="FILENAME OF THE SCRIPT.PY (make sure to include the .py if is python)" />

### How to add more machines to the launch file ###

Currently there are 4 PIs set up correctly i think its just the Rover2 which needs to be added. To do this it must be done at the top of the file. It needs a name which is essentially the variable name and the IP address of the PI. This can be found by useing the command.

hostname -I

Then just copy the other ones that are there already and only change these two catagries.

### How to configure new machines ###

BEFORE THE LAUNCH FILE WILL WORK WITH MORE MACHIENES THESE CONFIGUREATION STEPS MUST BE DONE

From the local machine type

ssh -oHostKeyAlgorithms='ssh-rsa' gee301@IP_ADDRESS_HERE

This allows for ros to ssh into the raspberry pi

On the Raspberry Pi

sudo nano /etc/hosts

add the IP addresss and hostname of the pc under the localhost the hostname is uos-OptiPlex-7060 the ip is 143.167.46.172

ctrl+s
ctrl+x

sudo nano /opt/ros/melodic/env.sh

above the line

exec "$@"

add

source /home/gee301/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://uos-OptiPlex-7060:11311/
export ROS_HOSTNAME=PI_IP_ADDRESS 


### THIS IS CONDITIONAL TO THE IP ADDRESSES OF THE PIS WHICH MAY CHANGE AS THE ROTATE EVERY 30 DAYS. IN WHICH CASE THE ENV.SH AND THE LAUNCH FILE NEED TO BE UPDATED. ###

------------------------------------------------------------------------



### out of date but kept for interset incase things dont work again ###

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
