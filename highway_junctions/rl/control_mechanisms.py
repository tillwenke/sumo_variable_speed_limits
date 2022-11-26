import os, sys
from statistics import mean
import traci
import numpy as np
from matplotlib import pyplot as plt

TO_MPS = 5/18
TO_KMPH = 18/5

def lecture_mechanism(occupancy_desired, occupancy_old, flow_old, road_segments):
    # occupancy_desired from plot experiments
    K_r = 25 # mu veh/h/%

    flow_new = flow_old + K_r * (occupancy_desired - occupancy_old)

    # speeds are handled in m/s
    # speed limit should be same on all lanes of an edge
    speed_old = traci.lane.getMaxSpeed(road_segments[0][0])
    
    speed_change = 1.4 # 5 km/h

    if flow_new < flow_old:
        speed_new = speed_old - speed_change
    else:
        speed_new = speed_old + speed_change

    # keep speed in reasonable range
    if 14 < speed_new < 38:
        print('SPEED', speed_new)
        for segment in road_segments:
            [traci.lane.setMaxSpeed(lane, speed_new) for lane in segment]

# by Carlson et al. (2011) 
# first b is 1.0
# max speed in m/s
# for application segments, leave one acceleration segment free before the merge
# for occupancy_desired run simulation without control mechanism
def mtfc(occupancy, occupancy_desired, b_old, max_speed, application_segments):
    K_I = 0.005
    b_min = 0.2
    b_max = 1.0

    b_new = b_old + K_I * (occupancy_desired - occupancy)

    if b_min <= b_new <= b_max:
        # apply new speed limit
        speed_new = max_speed * b_new

        # rounding
        speed_new = speed_new * 3.6 # m/s to km/h
        speed_new = round(speed_new, -1) # round to tens        
        print('NEW SPEED', speed_new)
        speed_new = speed_new / 3.6 # km/h to m/s

        for segment in application_segments:
            [traci.lane.setMaxSpeed(lane, speed_new) for lane in segment]
        return b_new
    else:
        print('NO SPEED CHANGE')
        return b_old

def mcs(segments, default_speed: float):    

    default_speed = default_speed*TO_KMPH
    mean_speed = traci.edge.getLastStepMeanSpeed('seg_0_before')*TO_KMPH

    speed_limits = None
    target_segments = segments[-6:]

    if mean_speed <= 45:
        print('Adjusting speed limits.')
        speed_limits = [100, 100, 80, 80, 60, 60]
    else:
        print('Reseting speed limits.')
        speed_limits = [default_speed]*len(target_segments)

    # Update speed limits.
    for limit, seg in zip(speed_limits, target_segments):
        for lane in seg:
            traci.lane.setMaxSpeed(lane, limit*TO_MPS)


# another example for an easy/ naive (rule based ?) algorithm
    """
    if density_after < density_before:
        top = traci.lane.getMaxSpeed(seg_0_before_top)
        bottom = traci.lane.getMaxSpeed(seg_0_before_bottom)

        traci.lane.setMaxSpeed(seg_0_before_top, top + 0.1)
        traci.lane.setMaxSpeed(seg_0_before_bottom, bottom + 0.1)
    """

