#! /usr/bin/env python
import rospy
import time 

# Planner used for holding information about the current orders and deciding on actions to take
from Planner import Planner

from std_msgs.msg import String
#from std_msgs.msg import Float32
from std_msgs.msg import Bool


global jobsList
jobsList = []

# Create the planner object as a global variable to be used throughout the node.
global planner
planner = Planner()

# Before we have a GUI, orders will be inputted here at compile time
planner.addOrder()

def actionDecider (data):
    arenaState = data.data
    global jobsList 
    #Dan insert your prioritisation stuff here

    # Not sure what data is coming in as. Possibly a string
    # needs to be coverted into a dictionary (like the one in Arena_Class.py)

    # Convert data to a dictionary of states
    states = eval(arenaState)

    chosenAction = planner.DecideOnAction(states)

    # do job takes two arguments, explained below, 
    # set as appropreate and call doJob

    #JobType options 
    #dispenser_1,dispenser_2,dispenser_3 (1 = orange, 2 = blue, 3 = green)
    #blackbox_1,blackbox_2,blackbox_3
    #rover_1,rover_2
    #glue
    # JobType = "type"
    # .where not implemented yet. (14:41/5/5/22)
    JobType = chosenAction.where

    #data of importance, none needed for dispensers and black boxes, 
    #rover uses "startPos,endPos" being stations to start at and go to
    #glue uses "assemblyType" being 1,2,3
    # JobData = "relevent_data"
    JobData = chosenAction.operationData

    doJob (JobType,JobData)


def jobUpdater (station,condition):
    global jobsList
    # mark whatever station is as whatever the update is
    # Any actions whose .where is station will have their .status set to condition
    planner.stationUpdate(station, condition)
    pass


def doJob (JobType,JobData):
    # function that publishes the triggering data for any given station
    if JobType.split('_')[0] == "dispenser":
        dispensers[int(JobType.split('_')[1])-1].publish(True) # might not work, if doesnt replace with if statement. 
        
    elif JobType.split('_')[0] == "blackbox":
        blackboxes[int(JobType.split('_')[1])-1].publish(True) # might not work, if doesnt replace with if statement. 
    
    elif JobType.split('_')[0] == "rover":
        rovers[int(JobType.split('_')[1])-1].publish(JobData) # might not work, if doesnt replace with if statement. 
    
    elif JobType == "glue":
        glueCommand.publish(JobData)
    
    #updates the arena and job list with new allocated stations and inprogress operations
    update_Arena.publish(JobType+",allocated")
    jobUpdater(JobType,"inProgress")

    time.sleep(1)
    requestArenaUpdate.publish (True)


            

def jobDone (data,station):
    #updates the job actions to completed and returns any reseting commands to the stations
    if station.split('_')[0] == "dispenser":
        if data.data == True:
            dispensers[int(station.split('_')[1])-1].publish(False) # might not work, if doesnt replace with if statement.
            jobUpdater(station,"complete")
            
    elif station.split('_')[0] == "blackbox":
        jobUpdater(station,"complete")
        
    elif station.split('_')[0] == "rover":
        jobUpdater(station,"complete")
    
    elif station.split('_')[0] == "glue":
        jobUpdater(station,"complete")
        


            
            
if __name__ == '__main__':
    try:

        rospy.init_node('Action_Planner',anonymous = True) 
        
        #list of publishers         
        update_Arena = rospy.Publisher('Update_Arena',String,queue_size = 10)
        requestArenaUpdate = rospy.Publisher('Request_Arena_Data',Bool, queue_size = 10)
        
        dispenseCommand1 = rospy.Publisher('/Dispenser/Dispense_Command_1', Bool, queue_size = 10 ) 
        dispenseCommand2 = rospy.Publisher('/Dispenser/Dispense_Command_2', Bool, queue_size = 10 )
        dispenseCommand3 = rospy.Publisher('/Dispenser/Dispense_Command_3', Bool, queue_size = 10 )
        dispensers = [dispenseCommand1,dispenseCommand2,dispenseCommand3] 
        # stores dispensers as a list of objects to ease publishing to them
        
        blackboxCommand1 = rospy.Publisher('/Blackbox/Blackbox_Job_Request_1', Bool, queue_size = 10 ) 
        blackboxCommand2 = rospy.Publisher('/Blackbox/Blackbox_Job_Request_2', Bool, queue_size = 10 )
        blackboxCommand3 = rospy.Publisher('/Blackbox/Blackbox_Job_Request_3', Bool, queue_size = 10 )
        blackboxes = [blackboxCommand1,blackboxCommand2,blackboxCommand3]
        
        roverCommand1 = rospy.Publisher('/Rover_1/Rover_Move', String, queue_size = 10 ) 
        roverCommand2 = rospy.Publisher('/Rover_2/Rover_Move', String, queue_size = 10 )
        rovers = [roverCommand1,roverCommand2]
        
        glueCommand = rospy.Publisher("/GlueGun_Workstation/Glue_Station_Command",String, queue_size = 10)
        
        #list of subscribers
        rospy.Subscriber('/Blackbox/Blackbox_Job_Complete_1',Bool,jobDone,callback_args = "blackbox_1")
        rospy.Subscriber('/Blackbox/Blackbox_Job_Complete_2',Bool,jobDone,callback_args = "blackbox_2")
        rospy.Subscriber('/Blackbox/Blackbox_Job_Complete_3',Bool,jobDone,callback_args = "blackbox_3")
        
        rospy.Subscriber('/Dispenser/Dispense_Confermation_1', Bool,jobDone,callback_args = "dispenser_1")
        rospy.Subscriber('/Dispenser/Dispense_Confermation_2', Bool,jobDone,callback_args = "dispenser_2")        
        rospy.Subscriber('/Dispenser/Dispense_Confermation_3', Bool,jobDone,callback_args = "dispenser_3")    
        
        rospy.Subscriber('/Rover_1/Rover_Job_Complete', Bool,jobDone,callback_args = "rover_1")
        rospy.Subscriber('/Rover_2/Rover_Job_Complete', Bool,jobDone,callback_args = "rover_2") 
        
        rospy.Subscriber('/GlueGun_Workstation/Glue_Job_Complete',Bool, jobDone,callback_args = "glue")


        rospy.Subscriber('ArenaState',String,actionDecider)

        #spin command to keep listening whilst nothing is happening
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    

