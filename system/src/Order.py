# -*- coding: utf-8 -*-
"""
Created on Tue Mar  1 14:54:08 2022
@author: Daniel
"""
# uses datetime to keep track of when orders are due and how long left till
# the due time is.
from datetime import datetime as dt
from datetime import timedelta as dl

# isnan is used to check for nan values. Annoyingly, value == float("nan") does
# not work! This is used in the Order.getUrgencyGradient() function.
from math import isnan

import Job
    
# Order class for keeping track of orders. This includes information about 
# which actions need to be completed in which order, when the order is
# requested to be completed by.
class Order:
    
    ### WORK TO DO ###
    # Change up the init function so that it can take configurations instead
    # of job files.
        
    # creates an order object. Is fed a Job and the due time of this job.
    def __init__(self, inputJob = Job.Job(), finishTime = 2.0, orderName = "no name", description = "no description", orderer = "no orderer"):
        
        self.orderName = orderName
        self.description = description
        self.orderer = orderer
        
        # Set the input job. Input job can either be a job object or a filename
        # for a job recipe.
        if type(inputJob) == str:
            self.job = Job.Job(inputJob)
        else:
            assert (type(inputJob) == type(Job.Job())), "Input job should either be a Job object or a string filename."
            # Set this instance's job to be the inputted job with no
            # pre-processing.
            self.job = inputJob
        
        # Get the current time. This is done here rather than later (which
        # would be more elegant) because I (Dan) don't know how to talk types
        # in Python yet so am getting types using type(something of that type)
        currentTime = dt.now()
        
        # Finish time should be either a datetime object if you specify that
        # the order should be completed at 13:57 for example, or a float as a 
        # number of minutes ()
        # isAbsolute says whether it is given as an absolute time. If this
        # value is False, then (after the assert statement), we know that the 
        # inputted finishTime is given as relative instead of absoulute
        fType = type(finishTime)
        isAbsolute = fType == type(currentTime)
        isRelative = fType == float or fType == int

                
        # Exactly one of isRelative and isAbsolute should be true. ^ is XOR.
        assert isAbsolute ^ isRelative, \
            "finishTime should be a datetime type input or double value."
        
        # If the finish time is given as relative, convert this into an
        # absolute finish time. Is absolute is now true.
        if not(isAbsolute):
            finishTime = currentTime + dl(minutes = finishTime)
            # These are not stictly necessary as they are not used for the rest
            # of the constructor, but are left in incase the constructor is 
            # added to.
            isAbsolute = True
            isRelative = False
            
        # Now that finish time is an absolute datetime object,
        # RequestedFinishTime can be set.
        self.requestedFinishTime = finishTime
        
    
    # This function returns the 'urgency' of this job. This is represented as a
    # double. The number represents an estimate of how many seconds of
    # processing time needs to be completed per second in order to get the job
    # done on time. If the job is overdue, this will return float("inf").
    def getUrgency(self, time = None):
        # See setTime for a detailed explaination.
        time = Order.setTime(time)
        
        timeLeft = self.getTimeTillDue(time).total_seconds()
            
        # If the due date has passed. Set the urgency to infinity. Without
        # this, the urgency would be negative. float("Inf") works with
        # comparisons so be a functional value.
        # If there is >0 time left, then the urgency is calculated as it should
        # be.
        if timeLeft <= 0:
            return float("Inf")
        else:
            return (self.getRemainingProcessingTime()/timeLeft) 
    
    # This function will return the gradient of the urgency of the job: how
    # much the urgency will increase in a second.
    def getUrgencyGradient(self, time = None, delta = dl(microseconds = 1)):
        time = Order.setTime(time)
        urgencyDif = self.getUrgency(time + delta) - self.getUrgency(time - delta)
        
        if isnan(urgencyDif):
            return float("inf")
        else:
            return urgencyDif / (2*delta.total_seconds())
        
    
    # This defines the less-than operator. This makes it such that you can sort
    # orders by their urgency easily.
    # This cannot deal with times other than current time.
    def __lt__(self, other):
        if not(self.getUrgency() == other.getUrgency()):
            # If they are not equal: compare by their urgencies
            return self.getUrgency() > other.getUrgency()
        else:
            # As a tiebreaker, return which one has the smallest due date
            # If they are both inf urgency, this will return the one that has
            # been overdue the longest.
            # If they both just happen to be exactly the same urgency (VERY
            # unlikely, but still worth planning for), this will return the one
            # due soonest.
            return self.requestedFinishTime < other.requestedFinishTime
    
    # This calls the Job method, getRemainingTime(). This will sum up all of
    # the processing times of the unstarted actions in the job.
    def getRemainingProcessingTime(self):
        return self.job.getRemainingTime()
    
    # This function returns the time left until the job is supposed to be
    # completed, represented as a timedelta object.
    def getTimeTillDue(self, time = None):
        time = Order.setTime(time)
        return self.requestedFinishTime - time
    
    # Returns the result of getStartableActions called on this order's job.
    def getStartableActions(self):
        return self.job.getStartableActions()
    
    # Defines the behaviour when returned to the console. For a more detailed
    # description of the action, use print(Action) (defined below)
    def __repr__(self):
        return "Order: " + self.orderName
    
    # Defines the print(Action) behaviour.
    def __str__(self):
        return self.__repr__() + "\n" + self.description + "\nOrdered By: " + self.orderer + "\nDue At: " + str(self.requestedFinishTime) + "\nCurrent Urgency: " + str(self.getUrgency())
    
    # This function is useful for setting the time correctly for time based
    # functions where you want the default time to be (time = dt.now()), but
    # want this to be used for any time you ask (time = dt.tomorrow()). The
    # optional parameter's default value gets set once and never re-evaluated
    # if it is a function call.
    # The way to get it to automatically update is to have a line in your
    # function that says if time is some predefined value, set it to the value 
    # returned by a certain function. That is what the setTime function does.
    # To have dt.now() as a default argument, set time = None in your parameter
    # list and have the first line of the function be:
    #       time = Order.setTime(time)
    # which will handle the rest.
    # I'm not sure if there's a way to automate all of that but it's only a
    # couple of things to remember and won't need to be implemented much
    # anyway.
    @staticmethod
    def setTime(time):
        if time is None:
            return dt.now()
        else:
            return time
        
    def updateStation(self, station, condition):
        # pass the message down to the job
        self.job.updateStation(station,condition)
        