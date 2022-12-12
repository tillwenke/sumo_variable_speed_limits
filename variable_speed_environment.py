import os, sys
from statistics import mean
import traci
import numpy as np
from matplotlib import pyplot as plt
import control_algorithms
import pandas as pd

from simulation_utilities.road import *
from simulation_utilities.setup import *

# set the algorithm to be used
# baseline, mtfc, mcs
approach = 'baseline'

#run sumo simulation
traci.start(sumoCmd)

# ----------------------------------------------- VARIABLE SETTING -----------------------------------------------

low_speed = 15 # 50 km/h
speed_max = 33.33 # 120 km/h

# INTRODUCE METRICS
density = 0
flow = 0
mean_speed = 0

# overall road speed stuff
mean_edge_speed = np.zeros(len(edges)) # for each edge
mean_road_speed = 0
ms = [] # mean speeds

cvs_seg_time = []
for i in range(len(all_segments)):
    cvs_seg_time.append([])
print(cvs_seg_time)

emissions = np.zeros(len(edges)) # for each edge
emissions_over_time = [] # for each time step


occupancy = 0 # highest anywhere in segment before merge
num = 0

density_before = 0
# ~ think about: travel time as overall satisfaction metric

# metric accumulators
veh_time_sum = 0 # for vehicles passing a point
veh_space_sum = 0 # for vehicles passing an area
mean_speed_sum = 0

veh_space_before_sum = 0
occupancy_sum = 0
num_sum = 0
dens = []
occ = []
flw = []

# CONTROL MECHANISM PARAMETERS
#mtfc
b = 1.0
application_area = segments_before[9:10]
print(application_area)

# SIMULATION PARAMETERS
step = 0
aggregation_time = 30 # seconds - always aggregate the last 100 step to make decision in the present
previous_harm_speeds = [120] * int(len(segments_before) / 2)

# ----------------------------------------------- SIMULATION LOOP -----------------------------------------------

# run till all cars are gone
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep() 
    step += 1
    
    # GATHER METRICS FROM SENSORS    
    # for some it is important to average over the number of lanes on the edge

    # AFTER
    veh_space_sum += sum([traci.lanearea.getLastStepVehicleNumber(detector) for detector in detectors_after])
    veh_time_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_after])

    mean_speed = sum([traci.inductionloop.getLastStepMeanSpeed(loop) for loop in loops_after]) / len(loops_after)
    # speed -1 indicated no vehicle on the loop
    if mean_speed >= 0:
        mean_speed_sum += mean_speed

    emission_sum = 0
    for i, edge in enumerate(edges):   
        mean_edge_speed[i] += traci.edge.getLastStepMeanSpeed(edge)
        emission_sum += traci.edge.getCO2Emission(edge)
    
    emissions_over_time.append(emission_sum)
    

    # BEFORE
    veh_space_before_sum += sum([traci.lanearea.getLastStepVehicleNumber(detector) for detector in detectors_before])

    # collecting the number of vehicles and occupancy right in front (or even in) of the merge area
    # choose max occupancy of a few sensors
    occ_max = 0
    for loops in loops_before:
        occ_loop = sum([traci.inductionloop.getLastStepOccupancy(loop) for loop in loops]) / len(loops)
        if occ_loop > occ_max:
            occ_max = occ_loop

    occupancy_sum += occ_max
    num_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_before[0]]) # only at one sensor
    
    # EVALUATE THE METRICS FREQUENTLY AND USE TO ADJUST VARIABLE SPEED LIMITS
    if step % aggregation_time == 0:

        # collected metrics are devided by the aggregation time to get the average values
        # OVERALL
        mean_edge_speed = mean_edge_speed / aggregation_time # first is acutally a sum
        mean_road_speed = sum(mean_edge_speed) / len(mean_edge_speed)
        ms.append(mean_road_speed)

        # AFTER THE MERGE
        density = ((veh_space_sum / aggregation_time) / detector_length) * 1000
        flow = (veh_time_sum / aggregation_time) * 3600
        flw.append(flow)
        mean_speed = (mean_speed_sum / aggregation_time) * 3.6 # one speed metric is enough - equals to spot speed

        # BEFORE THE MERGE
        density_before = ((veh_space_before_sum / aggregation_time) / detector_length) * 1000
        dens.append(density_before)

        occupancy = occupancy_sum / aggregation_time
        occ.append(occupancy)
        num = num_sum / aggregation_time

        # monitor the safety of road segments (CVS) - stores cvs value for each segment for each aggregation time step
        for i, seg in enumerate(all_segments):
            print(i, ':', seg)
            cvs_sum = 0
            for lane in seg:
                # for cvs
                ids = traci.lane.getLastStepVehicleIDs(lane)
                speeds = []
                for id in ids:
                    speeds.append(traci.vehicle.getSpeed(id))
                speeds = np.array(speeds)
                lane_avg = np.mean(speeds)
                lane_stdv = np.std(speeds)
                cvs_sum += lane_stdv / lane_avg
            cvs_seg = cvs_sum / len(seg)
            if np.isnan(cvs_seg):
                cvs_seg = 0
            cvs_seg_time[i].append(cvs_seg)
        
        # CONTROL MECHANISM - VARIABLE SPEED LIMIT ALGORITHM    

        if approach == 'mtfc':
            b = control_algorithms.mtfc(occupancy, 12, b, speed_max, application_area)
        elif approach == 'mcs':
            previous_harm_speeds = control_algorithms.adjusted_mcs(segments_before, speed_max, previous_harm_speeds)
        else:
            pass        

        # reset accumulator
        veh_time_sum = 0
        veh_space_sum = 0
        mean_speed_sum = 0

        mean_edge_speed = np.zeros(len(edges))
        mean_road_speed = 0

        veh_space_before_sum = 0
        occupancy_sum = 0
        num_sum = 0

# Save metrics into csv file.
with open(f'./metrics/{approach}_metrics.csv', 'w+') as metrics_file:
    list_to_string = lambda x: ','.join([ str(elem) for elem in x ]) + '\n'
    metrics_file.write(list_to_string(ms))
    metrics_file.write(list_to_string(flw))
    metrics_file.write(list_to_string(emissions_over_time))

# plot occupancy and flow diagram to get capacity flow    
fig, ax = plt.subplots(1,1, figsize=(15,30)) 
plt.xticks(np.arange(min(occ), max(occ)+1, 1.0))
plt.plot(occ, flw, 'bo')
plt.show() 

pd.DataFrame(cvs_seg_time).to_csv(f'./metrics/{approach}_cvs.csv', index=False, header=False)

# plot other metrics
plt.plot(ms)
plt.show()
plt.plot(emissions_over_time)
plt.show()
plt.plot(flw)
plt.show()

traci.close()