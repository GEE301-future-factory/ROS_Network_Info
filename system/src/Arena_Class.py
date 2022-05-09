# -*- coding: utf-8 -*-
"""
Created on Tue Apr  5 14:13:35 2022
@author: Daniel
"""

# import Locations

class Arena:
    
    # I can't remember what this was going to be after coming back from Easter 
    # break.
    # Maybe the Locations class I had half-implemented?
    # locations = Locations.Locations()
    
    # Black Boxes
    BB1 = "blackbox_1"
    BB2 = "blackbox_2"
    BB3 = "blackbox_3"
    
    # 3 Dispensers
    Disp_Red = "dispenser_1"
    Disp_Green = "dispenser_2"
    Disp_Blue = "dispenser_3"
    
    # Handover
    Handover = "Handover Station"
    
    # Glue-Station
    Glue_Station = "glue"
    
    # Rovers
    Rover1 = "rover_1"
    Rover2 = "rover_2"
    
    # Possible States
    Free = "free"
    Busy = "inProgress"
    Done = "complete"
    
    # Bug Fixer
    Start = "start"
    
    def __init__(self):
        # States is a dictionary of all the hardware (things) in the arena and
        # what state they are in.
        self.states = {
            Arena.BB1           : Arena.Free,
            Arena.BB2           : Arena.Free,
            Arena.BB3           : Arena.Free,
            Arena.Disp_Red      : Arena.Free,
            Arena.Disp_Green    : Arena.Free,
            Arena.Disp_Blue     : Arena.Free,
            Arena.Handover      : Arena.Free,
            Arena.Glue_Station  : Arena.Free,
            Arena.Rover1        : Arena.Free,
            Arena.Rover2        : Arena.Free,
            # Here just as a quick fix way to get move actions to work.
            Arena.Start         : Arena.Done
            }