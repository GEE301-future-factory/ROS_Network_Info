#! /usr/bin/env python
import rospy

# Import the arena used to store information about the current arena state.
from Arena_Class import Arena

from std_msgs.msg import String
#from std_msgs.msg import Float32
from std_msgs.msg import Bool
from system.msg import Handover

# current state of arena
global ArenaState
ArenaState = "data"

global arena
arena = Arena()

#updates the database of station conditions, (called by all subsequent updaters)
def ArenaUpdater(station, condition):
    global ArenaState
    #update the database  
    # ArenaState = str(station+condition) Dan commented out what jimmy wrote because he thought he knew what he was doing
    arena.states[station] = condition
    pass

#updates as stations are allocated
def AllocatedStationUpdater (data):
    station = data.data.split(',')[0]
    update = data.data.split(',')[1]
    ArenaUpdater(station,update)

#updates the state of free stations
def FreeStationUpdater (data,station):
    if station.split('_')[0] == "dispenser":
        if data.data == False:
            ArenaUpdater(station,"free")
            
    elif station.split('_')[0] == "blackbox":
        if data.data == False:
            ArenaUpdater(station,"free")
        
    elif station.split('_')[0] == "rover":
        ArenaUpdater(station,"free")
    
    elif station.split('_')[0] == "glue":
        ArenaUpdater(station,"free")

# updates the state of the handover station        
def HandoverUpdater (data):
    if data.robot == True:
        ArenaUpdater("handover","free")
    elif data.travel == True:
        ArenaUpdater("handover","allocated")
    elif data.human == True:
        ArenaUpdater("handover","allocated")


# publishes the arena state when it is asked for 
def SendArenaData (data):
    global ArenaState
    ArenaInfo.publish(str(arena.states))


if __name__ == '__main__':
    try:
        rospy.init_node('Arena',anonymous = True) 
        
        #subscribers        
        rospy.Subscriber('/Blackbox/Block_Present_1',Bool,FreeStationUpdater,callback_args = "blackbox_1")
        rospy.Subscriber('/Blackbox/Block_Present_2',Bool,FreeStationUpdater,callback_args = "blackbox_2")
        rospy.Subscriber('/Blackbox/Block_Present_3',Bool,FreeStationUpdater,callback_args = "blackbox_3")
        
        rospy.Subscriber('/Dispenser/Dispense_Confermation_1', Bool,FreeStationUpdater,callback_args = "dispenser_1")
        rospy.Subscriber('/Dispenser/Dispense_Confermation_2', Bool,FreeStationUpdater,callback_args = "dispenser_2")        
        rospy.Subscriber('/Dispenser/Dispense_Confermation_3', Bool,FreeStationUpdater,callback_args = "dispenser_3")    
        
        rospy.Subscriber('/Rover_1/Rover_Job_Done', Bool,FreeStationUpdater,callback_args = "rover_1")
        rospy.Subscriber('/Rover_2/Rover_Job_Done', Bool,FreeStationUpdater,callback_args = "rover_2") 
        
        rospy.Subscriber('/GlueGun_Workstation/Glue_Job_Done',Bool, FreeStationUpdater,callback_args = "glue")
        
        rospy.Subscriber('/Handover/Handover_data',Handover,HandoverUpdater)
        
        rospy.Subscriber('Update_Arena',String, AllocatedStationUpdater)
        rospy.Subscriber('Request_Arena_Data',Bool,SendArenaData)

        ArenaInfo = rospy.Publisher('ArenaState',String, queue_size= 10)
        
        #sping command 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    