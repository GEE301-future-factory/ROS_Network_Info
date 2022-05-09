# -*- coding: utf-8 -*-
"""
Created on Wed Mar  2 18:20:38 2022
@author: Daniel
"""    
import Action
from Arena import Arena as AR

class Job:
    
    # Work to do:
        # Change up __init__() to accept Josh's GUI output and convert that
        # into the action graph.
        # ConfigTxt -> prints,blackbox,glue ->
        # prints,moves,blackboxes,moves,expandedGlue,move,handover.
        
    
    def __init__(self, GUIConfig = "babytest"):
        # Initialise the actions dictionary as empty. Adding actions to the 
        # dictionary should be handled by the addAction() function.
        # Create the start and end actions. These are used as bookends for the
        # job description. Start's status is initiated as "completed".
        # Before the rest of the job is made, the job is just start -> end.
        self.makeStartAndEnd()
        
        # First dimention stores the links.
        # Second dimention stores: [precedes, sucedes].
        # Many to one and one to many will all be encapsulated by multiple links.
        # The start and end links are made here instead of in the addAction()
        # function because addAction() requires that you inject your action
        # between two already existing actions (which must always be the case,
        # given you have "start" and "end" actions). The link is created here 
        # manually instead.
        self.links = [["start", "end"]]
        
        # Convert the GUI config into the action list.
        configInfo = GUIConfig.split("|")
        
        # In the GUI GitHub the example is given as having a | at the end.
        # Not sure if this is a mistake or whether this is it will actually
        # come, but this fixes it if it needs fixing.
        if configInfo[len(configInfo)-1] == '':
            configInfo.pop(len(configInfo) - 1)
        
        if configInfo[0] == "babytest":
            self.addAction("Print_testblock","start", "end", "Blue")
        elif configInfo[0] == '2.1':
            raise NotImplementedError
        elif configInfo[0] == '3.1':
            raise NotImplementedError
        elif configInfo[0] == '3.2':
            self.addAction("Glue_2","start","end",1)
            self.addAction("Print_M3","start","Glue_2",configInfo[3])
            self.addAction("Glue_1","start","Glue_2",1)
            self.addAction("Move","Print_M3","Glue_2")
            self.addAction("Print_M1","start","Glue_1",configInfo[1])
            self.addAction("Print_M2","start","Glue_1",configInfo[2])
            self.addAction("Move", "Print_M1", "Glue_1")
            self.addAction("Move", "Print_M2", "Glue_1")
        else:
            raise NotImplementedError
        

    # This function adds a new action to the job. Links are managed by this
    # function.
    def addAction(self, name, befores = [], afters = [],
                  operationData = None):
        # Create the action instance for this new action and add it to the
        # actions dictionary. Action(name) creates the action instance.
        
        # If befores and afters aren't given as lists, convert them to lists.
        if type(befores) != list:
            befores = [befores]
        if type(afters) != list:
            afters = [afters]
        
        opType = name.split("_")[0]
        if opType == "BlackBox":
            self.actions[name] = Action.BlackBox(name)
        if opType == "Print":
            self.actions[name] = Action.Dispenser(
                name = name,
                inputColour = operationData)
        if opType == "Glue":
            self.actions[name] = Action.GlueStation(
                name = name,
                blockConfig = operationData)
        if opType == "Move":
            # Move will need a bit of work to make work properly.
            assert len(befores) < 2, "befores must be 0 or 1 length"
            assert len(afters) == 1, "afters must be 1 length"
            
            if len(befores) == 0:
                fromLoc = "None"
            elif len(befores) == 1:
                fromLoc = befores[0]

            toLoc = afters[0]
            
            self.actions[name] = Action.Move(name, fromLoc, toLoc)
                
        # Messiest implementation
        for before in befores:
            self.links.append([before,name])
        for after in afters:
            self.links.append([name,after])
    
    # This function returns the an estimate for the amount of processing time
    # left for the the job to be completed. This is currently a slightly crude
    # implementation but has room for improvement if necessary.
    # This will return a variable of type Double (probably).
    def getRemainingTime(self):       
        # Initialise the counter for the estimated processing time left for
        # this job to be completed as zero.
        remainingTime = 0
        
        # For all of the actions that must be taken to complete this job:
            # This is done by iterating through all of they keys in the
            # dictionary
        for key in self.actions.keys():
            
            # The action currently being looked at is accessed using the key.
            thisAction = self.actions[key]
            
            # If they have not been started, add their full estimated
            # processing time to the counter.
            if thisAction.status == "unstarted":
                increment = thisAction.estDuration
            else:
                increment = 0
            # For more complex remaining time estimates, add statements here
            # For example, an elif to check if it has been started and then add
            # 50% of estDuration to remaining time
                
            # Increment the remainingTime counter by however much increment has
            # been set to.
            remainingTime = remainingTime + increment
        
        # After summing the time left in the actions required to complete this
        # job, return the quantity.
        return remainingTime
    
    def getStartableActions(self, states):
        # Initialise as empty array. Add as actions are found.
        startables = []
        
        # Check every action.
        for action in self.actions:
            
            # easier to refer to thisAction than blahblah
            thisAction = self.actions[action]
            
            # Action is only startable if it is unstarted
            if thisAction.status == Action.Action.unstarted:
            
                # First check whether all preceds actions are completed.
                precedsDone = True
                # Check all links
                for link in self.links:
                    # if this action is the suceeds and the preceds is not completed
                    if (link[1] == action) and (self.actions[link[0]].status != Action.Action.complete):
                        # then not all preceds are completed
                        precedsDone = False
                        # and there's no need to check the rest of the links
                        break
                
                # if this action is doable given the current arena state and all
                # preceding actions are complete, add this action to the list of
                # doable actions
                
                # this is portrayed like this. The preceds check is done before the
                # isDoableAction check because for some actions (like move), all 
                # preceds actions need to be completed to know exactly how the
                # action should be completed (thus whether or not it is doable yet)
                if precedsDone:
                    
                    if thisAction.isDoableAction(states)[0]:
                        startables.append(thisAction)
        
        # return all found startable actions.
        return startables
    
    # Return all actions who's precedes have a status of "completed".
    def oldGetStartableActions(self, states = None):
        # this should instead check whether all the preceding links have been
        # completed and THEN check whether the action itself has been completed
        # as the move action requires knowledge about where the action before
        # it was completed before it knows whether it can be completed.
        
        # Initialise the startables list as empty
        startables = []
        
        # For each action:
        for action in self.actions:
            
            thisAction = self.actions[action]
            
            # An action can only be startable if it is unstarted
            if thisAction.status == Action.Action.unstarted:
                
                # Assume the action is startable, wait to be disproved by checking
                # all of the links
                
                # Set to action.isDoableAction(). If True, then it works as
                # before, if False, then it definitely ends as false. 
                isStartable = thisAction.isDoableAction(states)
                # If it finds a preceds action uncompleted, then the action
                # can't be completed.
                if isStartable:
                    for link in self.links:
                        # An action is startable if in all the links where it is the 
                        # succeds, the preceds is completed.
                        if (link[1] == action) and (self.actions[link[0]].status != Action.Action.complete):
                            # If this is the case, then the action being looked at is 
                            # proved to be unstartable.
                            isStartable = False
                            # Could put a break here to skip out of checking the
                            # remainder of the links, but likely to cause some bugs and
                            # not improve performance much, so not implemented yet.
                            # Room for improvement though!
                            # I did implement it in the end.
                            break
                if isStartable:
                    startables.append(thisAction)
        return startables
    
    def makeStartAndEnd(self):
        newActions = {}
        newActions["start"] = Action.Action.start()
        newActions["end"] = Action.Action.end()
        # Updated to make an instance method instead of a class method
        self.actions = newActions
            
    def updateStation(self, station, condition):
        # for all actions
        for action in self.actions:
            thisAction = self.actions[action]
            # pass the message down to the job
            thisAction.updateStation(station,condition)