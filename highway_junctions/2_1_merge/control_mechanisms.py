import os, sys
from statistics import mean
import traci
import numpy as np
from matplotlib import pyplot as plt


def lecture_mechanism(occupancy_desired, occupancy_old, flow_old, road_segments):
    # occupancy_desired from plot experiments
    K_r = 25 # mu veh/h/%

    flow_new = flow_old + K_r * (occupancy_desired - occupancy_old)

    # speeds are handled in m/s
    # speed limit should be same on all lanes of an edge
    speed_old = traci.lane.getMaxSpeed(road_segments[0])
    
    speed_change = 1.4 # 5 km/h

    if flow_new < flow_old:
        speed_new = speed_old - speed_change
    else:
        speed_new = speed_old + speed_change

    # keep speed in reasonable range
    if 14 < speed_new < 38:
        print('SPEED', speed_new)
        (traci.lane.setMaxSpeed(lane, speed_new) for lane in road_segments)   

