# -*- coding: utf-8 -*-
"""
Created on Wed May  4 12:55:13 2022
@author: Daniel
"""

# Copy of Action.py so I can rework it after the ROS update.

from Arena import Arena as AR

class Action:
    
    unstarted = "unstarted"
    complete = "complete"
    inProgress = "in progress"
    
    def __init__(self, inputName, inputEstDuration, inputDesc):
       self.name = inputName
       self.estDuration = inputEstDuration
       self.description = inputDesc
       self.status = Action.unstarted
       # If the object has no operation data, it still has the variable but
       # the value is none. this is to stop errors being thrown from not
       # finding the variable.
       self.operationData = None
       
       # This is so that the planner knows where the action thinks it is
       # being done. Set during isDoableAction
       self.where = None
       
    # Specialised constructors for the start and end methods.
    @classmethod
    def start(cls):
        startAc = Action("start", 0, "this is the start action")
        startAc.completeAction()
        return startAc
    
    @classmethod
    def end(cls):
        endAc = Action("end", 0, "this is the end action")
        return endAc
   
    def completeAction(self):
       self.status = Action.complete
   
    def setInProgress(self):
        self.status = Action.inProgress
       
    def isDoableAction(self, states):
        if self.name == "end" or self.name == "start":
            return [False, None]
        else:
            raise NotImplementedError(str(self) + " has no implementation")
            
    def updateStation(self, station, condition):
        # This gets passed to all actions in the planner.
        # If I had more time I'd make this better but this is the quickest fix
        if self.station == self.where:
            self.status = condition
            
    # Defines the behaviour when returned to the console. For a more detailed
    # description of the action, use print(Action) (defined below)
    def __repr__(self):
        return "Action: " + self.name
    
    # Defines the print(Action) behaviour.
    def __str__(self):
        return self.__repr__() + "\n" + self.description + "\nStatus: " + self.status
    
    # Defines the less than behaviour. Shorter time actions are first.
    def __lt__(self, other):
        return self.estDuration < other.estDuration

class BlackBox(Action):
    def __init__(self, inputName, inputEstDuration = 10,
                 inputDesc = "perfom a black box operation"):
        self.name = inputName
        self.inputEstDuration = inputEstDuration
        self.operationData = None
        self.where = None
        # Should be updated so that the description changes when where gets allocated
        # Will always say None even if it is.
        self.description = "Black box operation at station " + self.where
        self.status = Action.unstarted

        # Super is not compatible with python2, which is what ROS wants.
        # super(type(BlackBox), self).__init__(inputName, inputEstDuration, inputDesc)
        
    # States is a dictionary of hardware keys storing whether they are
    # free or not.
    def isDoableAction(self, states):
        if states[AR.BB1] == AR.Free:
            self.where = AR.BB1
            return True
        elif states[AR.BB2] == AR.Free:
            self.where = AR.BB2
            return True
        elif states[AR.BB3] == AR.Free:
            self.where = AR.BB3
            return True
        else:
            self.where = None
            return False

# For when writing the report, say inputEstDuration will be predicted using ML
class Dispenser(Action):
    def __init__(self, name, inputColour):
        self.name = name
        # inputColour should be a string. If you spell the colour wrong, it'll be green. Enjoy
        self.colour = inputColour
        self.inputEstDuration = 5
        self.description = "print a " + inputColour + " colour block"
        self.status = Action.unstarted
        self.operationData = None
        self.where = None
        
        # This was deleted as super was not working with Python2.
        # super(type(Dispenser), self).__init__(name, inputEstDuration = 5,
        #                 inputDesc = "print a " + inputColour + " colour block")

        
    
    def isDoableAction(self, states):
        if self.colour == "Orange":
            arenaName = AR.Disp_Red
        elif self.colour == "Green":
            arenaName = AR.Disp_Green
        elif self.colour == "Blue":
            arenaName = AR.Disp_Blue
        else:
            # IF YOU GET HERE YOU DIDN'T TYPE IN A VALID COLOUR
            # DEFAULT TO GREEN
            arenaName = AR.Disp_Green
        if states[arenaName] == AR.Free:
            self.where = arenaName
            return True
        else:
            self.where = None
            return False

class HandoverStation(Action):
    def isDoableAction(self, states):
        self.where = AR.Handover
        return True

class GlueStation(Action):
    def __init__(self, name, blockConfig):
        self.name = name
        self.estDuration = 60
        self.description = "I am a glue operation. Blob here: " + blockConfig
        self.status = Action.unstarted
        self.where = None
        self.operationData = blockConfig
    
    def isDoableAction(self, states):
        if states[AR.Glue_Station] == AR.Free:
            self.where = AR.Glue_Station
            return True
        else:
            self.where = None
            return False

class Move(Action):
    def __init__(self, name, fromLoc = "none", toLoc = "none"):
        super(type(Action), self).__init__(name, inputEstDuration = 60,
                         inputDesc = "move from " + fromLoc + " to " + toLoc)
        
        self.operationData = [fromLoc, toLoc]
    
    def isDoableAction(self, states):
        if (states[self.operationData[0]] == AR.Done):
            if states[AR.Rover1] == AR.Free:
                self.where = AR.Rover1
                return True
            elif states[AR.Rover2] == AR.Free:
                self.where = AR.Rover2
                return True
            else:
                self.where = None
                return False
        else:
            self.where = None
            return False
            
            
            
            