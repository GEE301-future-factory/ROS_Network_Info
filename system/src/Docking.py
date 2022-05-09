#! /usr/bin/env python

import rospy
from system.msg import LIDAR_Docking
from system.msg import moveblockdata
from std_msgs.msg import String
from std_msgs.msg import Bool
import time
import math

global direction
global relativeAngle
global distance
global length



def manouver (speed,directionDesired,manouverTime):
    #rover operation
    debugTop.publish("entered manourver")
    roverCommand.publish (speed)
    roverCommand.publish (directionDesired)
    roverCommand.publish ("go")
    time.sleep (manouverTime)
    roverCommand.publish ("stop") 

def dataNormalised(data):
#    debugTop.publish("entered data stored (1)")
    global direction
    global relativeAngle
    global distance
    global length
    
#    debugTop.publish("Got past the declarations (2)")
    
    #data import
    dataArray = data.data.split(',')

#    debugTop.publish("Converted the input to an array (3)")

    direction = float(dataArray[2])

#    debugTop.publish("Direction read (4): " + str(direction))

    relativeAngle = float(dataArray[3])

#    debugTop.publish("relativeAngle read (5): " + str(relativeAngle))

    distance = float(dataArray[1])

#    debugTop.publish("distance read (6): " + str(distance))

    length = float(dataArray[0])

#    debugTop.publish("length read (7): " + str(length))

#    debugTop.publish("Got past the data import and conversion (8)")
 #   debugTop.publish(str(dataArray))

    # data normaliseation to rover where 0 is forward and -90 is perfectly sideways, and -180 is backwards. normaliseation origin is 15cm backwards from the center of the lidar
    roverOffset = -0.15
    x = distance*math.sin(math.radians(90-direction))
    
#    debugTop.publish("Got past setting x (9)")

    y = distance*math.sin(math.radians(direction))

#    debugTop.publish("Got past setting y (10)")

    distance = math.sqrt(math.pow(x-roverOffset,2)+math.pow(y,2))

#    debugTop.publish("Got past setting distance (11)")

    direction = math.degrees(math.atan2(y,x-roverOffset))
#    debugTop.publish(str(direction))
#    debugTop.publish("dataStored")



def denormaliseExpectedDirection(direction,distance):

    # data normaliseation to rover where 0 is forward and -90 is perfectly sideways, and -180 is backwards. normaliseation origin is 15cm backwards from the center of the lidar
    roverOffset = 0.15
    x = distance*math.sin(math.radians(90-direction))
    y = distance*math.sin(math.radians(direction))
    distance = math.sqrt(math.pow(x+roverOffset,2)+math.pow(y,2))
    direction = math.degrees(math.atan2(y,x+roverOffset)) 
    return direction
    


def docking (data):
    
    global direction 
    global relativeAngle
    global distance
    global length   
#    rospy.Subscriber('docking_data', LIDAR_Docking , dataNormalised)
    
    debugTop.publish("docking triggered")
    #rover speed data
    rotationSpeedFast = 26 #degrees per second
    forwardBackSpeedSlow = 0.074 #meters per second 
    sidewaysSpeedSlow = 0.055 #meters per second 
    


    #manouver selection -------------------------------------------------------------------
    #critera
    parallelTolerance = 10 #degrees
    alignedTolerance = 10 #degrees#
    alignedAngle = -90 #degrees
    distanceTolerance = 0.05 #meters
    dockingDistance = 0.15 #meters

    while True:
        debugTop.publish("loop entered")
        #conditions 

        if  not(math.sin(math.radians(parallelTolerance))>math.sin(math.radians(relativeAngle))): # checks if not parallel
            debugTop.publish("Angle off if entered")
            if relativeAngle < -90: #direction of manourver and magnitude
                offsetFromDesired = 180-relativeAngle 
                directionDesired = "rotate_R"
            else:
                offsetFromDesired = relativeAngle 
                directionDesired = "rotate_L"
            manouverTime = abs(offsetFromDesired) / rotationSpeedFast  #calculates the time for the manouver
            speed = "fast" 
            #provides expected direction and distance
            expectedDirection = direction-offsetFromDesired
            expectedDistance = distance 

        elif not((alignedAngle-alignedTolerance) < direction < (alignedAngle+alignedTolerance)): # checks if not aligned
            debugTop.publish("forward back alignged entered")
            offsetFromDesired = distance*math.sin(math.radians(90-direction)) # if negative go backwards
            debugTop.publish(str(offsetFromDesired))
            manouverTime = abs(offsetFromDesired) / forwardBackSpeedSlow  #calculates the time for the manouver
            if offsetFromDesired > 0: #direction of manourver
                directionDesired = "forward"
            else:
                directionDesired = "back"
            speed = "slow"
            #provides expected direction and distance
            expectedDirection = alignedAngle
            expectedDistance = distance

        elif not((dockingDistance - distanceTolerance) < distance < (dockingDistance + distanceTolerance)): # checks if close enough 
            debugTop.publish("side aligned entered")
            offsetFromDesired = dockingDistance + distance*math.sin(math.radians(direction)) # if negative go right
            debugTop.publish(str(offsetFromDesired))
            manouverTime = abs(offsetFromDesired) / sidewaysSpeedSlow  #calculates the time for the manouver
            if offsetFromDesired > 0: #direction of manourver
                directionDesired = "left"
            else:
                directionDesired = "right"
            speed = "slow"
            #provides expected direction and distance
            expectedDirection = alignedAngle
            expectedDistance = dockingDistance
	    
        else: # if its docked send information to uarm and leave loop
            #output for UArm -----------------------------------------------------------------
            #armOutput = moveblockdata()
            #armOutput.move = True
            #armOutput.x = distance*math.sin(math.radians(90-direction))
            #armOutput.y = distance*math.sin(math.radians(direction))
            #armOutput.theta = relativeAngle 
            #dockingComplete.publish(armOutput) #output to the UArm
            debugTop.publish("do block pickup")
            break # leaves infinate loop

        debugTop.publish("end of if reached")
        manouver(speed,directionDesired,manouverTime)
        newScanRequest.publish(length+","+str(denormaliseExpectedDirection(expectedDirection,expectedDistance))) # updates scan search parameters
        time.sleep (1)
         


if __name__ == '__main__':
    rospy.init_node('DockingControler', anonymous=True)
    roverCommand = rospy.Publisher('/Rover_1/Motor_Control_1', String, queue_size=28) # rover control topic, publishes commands too
    newScanRequest = rospy.Publisher('/Local/search_data', String, queue_size=28) # publishes to the search data requesting a update on position
    dockingComplete = rospy.Publisher('/Rover_1/Move_Block_Data', LIDAR_Docking, queue_size=28) # output to Uarm giving the position of the rover 
    debugTop = rospy.Publisher('/Local/Not_Working', String, queue_size=28)
    rospy.Subscriber('/Rover_1/docking_data', String , dataNormalised) # this is the input from the lidar, should update before every loop
    rospy.Subscriber('/Rover_1/docking_Command', Bool, docking) #when overhead camara publishes to this it should trigger docking
    rospy.spin()

