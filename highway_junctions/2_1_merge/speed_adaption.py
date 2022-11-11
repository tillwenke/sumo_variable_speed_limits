import os, sys
from statistics import mean
import traci
import numpy as np
from matplotlib import pyplot as plt
import control_mechanisms


# from https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html

#setup
if 'SUMO_HOME' in os.environ:
     tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
     sys.path.append(tools)
else:
     sys.exit("please declare environment variable 'SUMO_HOME'")


#init sumo simulation
# -d, --delay FLOAT  Use FLOAT in ms as delay between simulation steps
sumoBinary = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "2_1_merge.sumocfg", '--start']

#run sumo simulation
traci.start(sumoCmd)

# ----------------------------------------------- VARIABLE SETTING -----------------------------------------------

# LANES & MAXIMUM SPEEDS
# naming:
# laneSEGment_"number, 0 means closest to merge zone"_"before or after the merge"_"lane number counting from bottom to top"
seg_0_before = ["seg_0_before_1", "seg_0_before_0"]
seg_0_after = ["seg_0_after_0"]

low_speed = 15 # 50 km/h
high_speed = 33.33 # 120 km/h

(traci.lane.setMaxSpeed(lane, high_speed) for lane in seg_0_before)
(traci.lane.setMaxSpeed(lane, high_speed) for lane in seg_0_after)

# ROAD SENSORS / INDUCTION LOOPS
# defined in additional.xml
# naming:
# equal to lanes
# keeping to the sumo objects used
# loop = induction loop ... for measurements at a point
# detector = lane are detector ... for measurements along a lane

loops_before = ["loop_seg_0_before_1", "loop_seg_0_before_0"]

loops_after = ["loop_seg_0_after_0"]
detectors_after = ["detector_seg_0_after_0"]

detector_length = 50 #meters

# INTRODUCE METRICS
density = 0
flow = 0
mean_speed = 0

occupancy = 0
num = 0
# ~ think about: travel time as overall satisfaction metric

# metric accumulators
veh_time_sum = 0
veh_space_sum = 0
mean_speed_sum = 0

occupancy_sum = 0
num_sum = 0
occ = []
flw = []

# SIMULATION PARAMETERS
step = 0
aggregation_time = 100 # seconds - always aggregate the last 100 step to make decision in the present

# ----------------------------------------------- SIMULATION LOOP -----------------------------------------------

# run till all cars are gone
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep() 
    step += 1

    # GATHER METRICS FROM SENSORS    
    # for some it is important to average over the number of lanes on the edge

    # AFTER
    veh_space_sum += sum([traci.lanearea.getLastStepVehicleNumber(detector) for detector in detectors_after])
    veh_time_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_before])

    mean_speed = sum([traci.inductionloop.getLastStepMeanSpeed(loop) for loop in loops_after]) / len(loops_after)
    # speed -1 indicated no vehicle on the loop
    if mean_speed >= 0:
        mean_speed_sum += mean_speed

    # BEFORE
    # collecting the number of vehicles and occupancy right in front (or even in) of the merge area
    occupancy_sum += sum([traci.inductionloop.getLastStepOccupancy(loop) for loop in loops_before]) / len(loops_before)
    num_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_before])
    
    # EVALUATE THE METRICS FREQUENTLY AND USE TO ADJUST VARIABLE SPEED LIMITS
    if step % aggregation_time == 0:

        # collected metrics are devided by the aggregation time to get the average values
        
        # AFTER THE MERGE
        density = ((veh_space_sum / aggregation_time) / detector_length) * 1000
        flow = (veh_time_sum / aggregation_time) * 3600
        flw.append(flow)
        mean_speed = (mean_speed_sum / aggregation_time) * 3.6 # one speed metric is enough - equals to spot speed
            
        print('density', round(density,3), 'veh/km ', end='; ')        
        print('flow', flow, 'veh/h', end='; ')        
        print('speed', round(mean_speed,2), 'km/h', end='; ')

        # BEFORE THE MERGE

        occupancy = occupancy_sum / aggregation_time
        occ.append(occupancy)
        num = num_sum / aggregation_time

        print('OCC', round(occupancy, 2))
        print('NUM', round(num, 2))
    
        
        # CONTROL MECHANISM - VARIABLE SPEED LIMIT ALGORITHM
        # implementation of https://www.sciencedirect.com/science/article/pii/S0968090X07000873        
        
        #control_mechanisms.lecture_mechanism(occupancy_desired=6, occupancy_old=occupancy, flow_old=flow, road_segments=seg_0_before)        
        
        # another example for an easy/ naive (rule based ?) algorithm
        """
        if density_after < density_before:
            top = traci.lane.getMaxSpeed(seg_0_before_top)
            bottom = traci.lane.getMaxSpeed(seg_0_before_bottom)

            traci.lane.setMaxSpeed(seg_0_before_top, top + 0.1)
            traci.lane.setMaxSpeed(seg_0_before_bottom, bottom + 0.1)
        """

        # reset accumulator
        veh_time_sum = 0
        veh_space_sum = 0
        mean_speed_sum = 0

        occupancy_sum = 0
        num_sum = 0

# plot occupancy and flow diagram to get capacity flow     
plt.scatter(occ, flw)
plt.show() 

traci.close()