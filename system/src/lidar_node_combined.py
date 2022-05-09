#! /usr/bin/env python
import math
from re import M
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from system.msg import LIDAR_Docking

def objectFinder(increment,ranges):
    
    ranges = ranges.replace('(','')
    ranges = ranges.replace(')','')
    ranges = ranges.split(', ')    
    
#setting a tolerance variable, the distance points must be apart to be considered as different objects
    tol = 0.03

    
#there needs to be a way to keep track of the last non-inf range when crossing many consecutive inf points
    lastGoodCounter = 0

#creating an array for the objects to be stored in, and a constantly changing array to store each objects length, angle and distance to midpoint
    objectArray = []
    objectStats = [0,0,0,0]

#variable assuming there is an object at the start of the scan
    startPolarCoords = [0,0-math.pi,0]
#for loop with a counter variable to check distance between each point - finding objects less than 1m away for docking purposes
    counter = 0

    startSew = False
    endSew = False

    for i in ranges:

        #arrays will return errors on the first and last points as there is no array[-1] or array[len+1]
        if counter !=0 and counter != (len(ranges) - 1):

            #to make sure only non-infinite ranges are used in the calculations, the final statement is so that only points less than 1m away will be considered
            if i != 'inf' and ranges[counter+1] != 'inf' and float(i)<1:

                #calculate angle of points, the lidar starts at -pi
                angle = counter*increment - math.pi

                #the distance between the current point and the last point within the correct range is calculated
                interDist = distCalc(ranges[lastGoodCounter],i,angle-increment*(counter-lastGoodCounter),angle)
                
                if counter == 1:
                    startSew = True
                
                #the distance just calculated must be above a tolerance length to prompt the program to declare a new object
                if interDist >= tol:

                    #this statement allows for the first object to be correctly captured
                    if lastGoodCounter == 0:
                        #print('New object found:')
                        
                        #the polar coordinates of the line's start point, as well as its counter, are stored for later calculation
                        if startSew == False:
                            startPolarCoords = [i,angle,counter]
                    #this statement applies to any point that isn't the first start point, or last end point    
                    else:
                        #polar coordinates of a line's end point are stored for calculation
                        endPolarCoords = [ranges[lastGoodCounter],angle-increment*(counter-lastGoodCounter),lastGoodCounter]
                            
                        if startSew == False:

                            
                            #length is calculated using start and end points in the distCalc function
                            objectLength = distCalc(startPolarCoords[0],endPolarCoords[0],startPolarCoords[1],endPolarCoords[1])

                            #midpoint variable is the counter variable of the median point in the line, rounded down for half values
                            midpoint = (endPolarCoords[2]+startPolarCoords[2])/2
                            midpoint=math.floor(midpoint)
                            midpoint = int(midpoint)

                            #the angle and distance of the line from the lidar are calculated using the line's midpoint;
                            #the angle is then recalculated as relative to the rover in degrees
                            lidarAngle = midpoint*increment - math.pi
                            lidarAngle += math.pi/2
                            if lidarAngle>math.pi:
                                lidarAngle -= math.pi*2
                            lidarAngle = lidarAngle*180/math.pi
                            objectDistance = float(ranges[midpoint])

                            #relative angle of line to the forwards direction of the rover is calculated
                            objectAngle = angleCalc(startPolarCoords[0],endPolarCoords[0],startPolarCoords[1],endPolarCoords[1])
                            
                            #objectStats arrays are stored in a 2D array
                            objectArray.append([objectLength,objectDistance,lidarAngle,objectAngle])

                            #print('Length:',objectLength,'m, at an angle of',objectAngle,'degrees;\nDistance:',objectDistance,'m, in the direction of',lidarAngle,'degrees.\n\n')
                            #print('New object found:')
                            
                            #the gap detected at the start of the if/else statement in use will be between the end point of one line, and the start of the next;
                            #start point of the next line is therefore recorded
                        
                        startPolarCoords = [i,angle,counter]
                        if startSew == True:
                            sewEndCoords = endPolarCoords
                            startSew = False
                            endSew = True
                #a separate counter is used to keep track of the previous point within the correct range
                lastGoodCounter = counter
                    
        #overall counter is increased by 1 each iteration of the loop            
        counter += 1
        
    #this statement is used to resolve the final object
    if counter == len(ranges):
        if endSew == False:
            #finds the end point using the last point within range and calculates the angle using the fact that at the end of the foor loop, the theoretical point in question would be at pi radians
            endPolarCoords = [ranges[lastGoodCounter+1],math.pi-increment*(counter-lastGoodCounter),lastGoodCounter]

            #objectStats calculated as before
            objectLength = distCalc(startPolarCoords[0],endPolarCoords[0],startPolarCoords[1],endPolarCoords[1])
            midpoint = (endPolarCoords[2]+startPolarCoords[2])/2
            midpoint=math.floor(midpoint)
            midpoint = int(midpoint)
            lidarAngle = midpoint*increment - math.pi
            lidarAngle += math.pi/2
            if lidarAngle>math.pi:
                lidarAngle -= math.pi*2
            lidarAngle = lidarAngle*180/math.pi
            objectDistance = float(ranges[midpoint])
            objectAngle = angleCalc(startPolarCoords[0],endPolarCoords[0],startPolarCoords[1],endPolarCoords[1])
            objectArray.append([objectLength,objectDistance,lidarAngle,objectAngle])
            #print('Length:',objectLength,'m, at an angle of',objectAngle,'degrees;\nDistance:',objectDistance,'m, in the direction of',lidarAngle,'degrees.\n\n')

        elif endSew == True:
            endPolarCoords = sewEndCoords
            #objectStats calculated as before
            objectLength = distCalc(startPolarCoords[0],endPolarCoords[0],startPolarCoords[1],endPolarCoords[1])
            startAngle = startPolarCoords[1]
            endAngle = endPolarCoords[1]
            startAngle += math.pi/2
            endAngle+=math.pi/2
            if startAngle>math.pi:
                startAngle -=math.pi*2
            if endAngle>math.pi:
                endAngle-=math.pi*2
            lidarAngle = (startAngle + endAngle) /2
            lidarAngle = lidarAngle*180/math.pi
            thetaOne = float(startAngle - endAngle)
            edgeDist = float(startPolarCoords[0])
            sinOne = float(math.sin(thetaOne/2))
            sinTwo = (edgeDist*sinOne)/(objectLength/2)

            # math.asin throws an error if input out of range [-1 : 1].
            if sinTwo < -1:
                sinTwo = -1
            elif sinTwo > 1:
                sinTwo = 1

            #debugTop.publish(str(sinTwo))
            thetaTwo = (math.asin(sinTwo))
            phi = math.pi-(thetaOne/2 + thetaTwo)
            objectDistance = (objectLength/2 * float(math.sin(phi)))/(sinOne)
            objectAngle = angleCalc(startPolarCoords[0],endPolarCoords[0],startPolarCoords[1],endPolarCoords[1])
            objectArray.append([objectLength,objectDistance,lidarAngle,objectAngle])
            endSew == False
            #print('Length:',objectLength,'m, at an angle of',objectAngle,'degrees;\nDistance:',objectDistance,'m, in the direction of',lidarAngle,'degrees.\n\n')
        
    return (objectArray)

        
#function to measure the angle between two points, given their polar coordinates - I use this between the end points of a line to measure its relative angle
#for reference, the relative angle to the lidar that the rover faces is -pi/2 radians
def angleCalc(d1,d2,a1,a2):
    coords1 = polarToCart (d1,a1)
    coords2 = polarToCart (d2,a2)
    
    #finds tangent of angle by doing y/x
    if coords2[0]-coords1[0] != 0:
        tAngle = (coords2[1]-coords1[1])/(coords2[0]-coords1[0])
    else:
        tAngle = 9999999999999999999999999999999999999999999999999999999999999
    
    #arctangent function
    angle = math.atan(tAngle)
    
    #calculates the angle relative to the rover zero
    angle += math.pi/2
    if angle>math.pi:
        angle -= math.pi*2
    angle = angle*180/math.pi
    return angle

def distCalc(d1,d2,a1,a2):
    coords1 = polarToCart(d1,a1)
    coords2 = polarToCart(d2,a2)

    #print (coords1,coords2)

    xDist = coords2[0]-coords1[0]
    yDist = coords2[1]-coords1[1]

    #print (xDist,yDist)
    sqDist = xDist**2 + yDist**2
    dist = float(math.sqrt(sqDist))
    return dist

    
#function to turn polar coordinates into cartesian coordinates
def polarToCart(d,a):
    d = float(d)
    a = float(a)
    x = d*math.cos(a)
    y = d*math.sin(a)
    coords = [x,y]
    return (coords)

angle = 0
length = 0

#function to use object finder tools to return an array of lines and their data
def searchData(search_data):
    data = search_data.data.split(',')
    global length, angle
    length = data[0]
    angle = data[1]
    


def data_return(scan):
    scan_data = (objectFinder(scan.angle_increment,str(scan.ranges)))
    #print(scan_data)
    locate(scan_data)
    


#function to locate objects of a certain length, in a certain field of direction, and return the distance, angle, and accurate direction
def locate(objects):

    
    
    global angle, length
    angle = float(angle)
    length = float(length)
    #print (length,angle)
    #tolerance for length - the lidar may see the length as slightly longer or shorter than it actually is
    lengthTol= float(0.02)
    
    dirTol = 20
    
    #defining arrays that will store line data

    #cycle through objects to find which ones fit the location criteria
    for i in objects:
        if (float(angle) - dirTol) < float(i[2]) < (float(angle) + dirTol):
            if float(i[0]) <= float(length) + lengthTol and float(i[0]) >= float(length) - lengthTol:

                #returns location parameters of station
                #dockingData = LIDAR_Docking()
                #dockingData.length = (i[0])
                #dockingData.distance = (i[1])
                #dockingData.angle = (i[3])
                #dockingData.direction = (i[2])
                # sent as the string repr of the array
                dataToSend = (str(i[0]) +','+ str(i[1]) +','+ str(i[2]) +','+ str(i[3]))
                pubLength.publish(dataToSend)

                #print(+','+stationDistance+','+stationAngle+','+stationDirection)
            else:
                #print ('no length')
                x=0
        else: 
            #print ('no direction')
            x=0 

if __name__ == "__main__":
    rospy.init_node('object_finder')  
    pubLength = rospy.Publisher('/Rover_1/docking_data',String, queue_size = 10) 
    #debugTop = rospy.Publisher('/Local/not_working',String, queue_size= 10)
    rospy.Subscriber('search_data',String,searchData)
    rospy.Subscriber('/Rover_1/scan',LaserScan,data_return)
    rospy.spin()
    
    
    
