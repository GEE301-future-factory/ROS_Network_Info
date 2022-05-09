# -*- coding: utf-8 -*-
"""
Created on Tue Mar 29 14:53:25 2022
@author: Daniel
"""

from Order import Order

class Planner:
    def __init__(self):
        self.orders = []
        
    # Add a new order to the planner.
    # Future updates include making this take a string text file and generate
    # the order within this function (like Order() does for Job (lines 22-30))
    def addOrder(self, newOrder = Order(finishTime = 5.0,
                 orderName = "Default Order", description = "This order was \
                 created by addOrder as no input order was given")):
        self.orders.append(newOrder)
        self.sortedByUrgency()
    
    # This function sorts the current orders by decending urgency (most urgent 
    # first, then decreasing urgency). The function also returns the sorted
    # list incase that is useful.
    def sortedByUrgency(self, arenaState = None):
        self.orders.sort(reverse = True)
    
    # This decides on which action to make
    def DecideOnAction(self, currentArenaState = None):
        
        # NEED TO IMPLEMENT PASS IF NO CURRENT ORDERS
        
        if len(self.orders) > 0:
        
            self.sortedByUrgency()
            
            # If most urgent order has no startable actions, check the 2nd most
            # urgent. I think I went with this in the end (dan)
            # OR
            # decide to wait out this calculation.
            for order in self.orders:
                startables = order.job.getStartableActions(currentArenaState)
                if len(startables) > 0:
                    startables.sort(reverse = True)
                    chosen = startables[0]
                    chosen.setInProgress()
                    return startables[0]
            assert True, "no actions were startable."
        print("Planner has no current orders.")
        # assert True, "Planner has no current orders."
    
    def stationUpdate(self, station, condition):
        # This function gets called when the arena says that the status of an 
        # action has changed. This function gets called to do that updating.
        
        # Pass the message down for all orders
        for order in self.orders:
            order.stationUpdate(station, condition)
                