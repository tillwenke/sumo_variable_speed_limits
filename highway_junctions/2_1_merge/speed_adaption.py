import os, sys
from statistics import mean
import traci
import numpy as np
from matplotlib import pyplot as plt


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
delay = '1.0'
#sumoCmd = [sumoBinary, "-c", "2_1_merge.sumocfg", "--step-length", delay, '--start'] #no need to specify path to sumocfg
sumoCmd = [sumoBinary, "-c", "2_1_merge.sumocfg", '--start']

#run sumo simulation
traci.start(sumoCmd)


# parameters and sumo objects

# define sumo objects
# lane 0 is at bottom
segment_length = 50 #meters

point_loops_before = ["point_loop_seg_0_before_1", "point_loop_seg_0_before_0"]
segment_loops_before =["segment_loop_seg_0_before_1", "segment_loop_seg_0_before_0"]

point_loops_after = ["point_loop_seg_0_after_0"]
segment_detectors_after = ["segment_detector_seg_0_after_0"]

# need those?
seg_0_before_bottom = "seg_0_before_1"
seg_0_before_top = "seg_0_before_0"
seg_0_after = "seg_0_after_0"

traci.lane.setMaxSpeed(seg_0_before_top, 10)
traci.lane.setMaxSpeed(seg_0_before_bottom, 10)

traci.lane.setMaxSpeed(seg_0_after, 10)



# define metrics
density = 0
flow = 0
mean_speed = 0
# finally travel time as overall satisfaction metric

# accumulators
occupancy_sum = 0
num_sum = 0
occ = []
flw = []

veh_on_sum = 0
veh_entered_sum = 0
mean_speed_on_sum = 0

veh_on_sum_b = 0
veh_entered_sum_b = 0
mean_speed_on_sum_b = 0

# variable speed limit
low_speed = 1
high_speed = 10

# simulation
step = 0
aggregation_time = 100 #seconds - always aggregate the last 100 step to make decision in the present
print('AVERAGED OVER', aggregation_time, 'STEPS')


# run till all cars are gone
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep() 
    step += 1

    # detection after
    veh_on = sum([traci.lanearea.getLastStepVehicleNumber(loop) for loop in segment_detectors_after])
    veh_on_sum += veh_on

    veh_entered = sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in point_loops_before])
    veh_entered_sum += veh_entered

    # "on" the induction loop
    mean_speed_on = sum([traci.inductionloop.getLastStepMeanSpeed(loop) for loop in point_loops_after]) / len(point_loops_after)
    # speed if -1 if no cars pass
    if mean_speed_on >= 0:
        mean_speed_on_sum += mean_speed_on

    # detection before
    veh_on_b = sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in segment_loops_before])
    veh_on_sum_b += veh_on_b

    veh_entered_b = sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in point_loops_before])
    veh_entered_sum_b += veh_entered_b

    # "on" the induction loop
    mean_speed_on_b = sum([traci.inductionloop.getLastStepMeanSpeed(loop) for loop in point_loops_before]) / len(point_loops_before)
    # speed if -1 if no cars pass
    if mean_speed_on_b >= 0:
        mean_speed_on_sum_b += mean_speed_on_b


    occupancy = sum([traci.inductionloop.getLastStepOccupancy(loop) for loop in point_loops_before]) / len(point_loops_before)
    occupancy_sum += occupancy

    num = sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in point_loops_before])
    num_sum += num

    # output
    # sanity check against the XML output
    """
    if step < 100:
        print(step-1, veh_on, '|', mean_speed_on)
    """
    
    if step % aggregation_time == 0:
        
        # after
        avg_num_veh_on = veh_on_sum / aggregation_time
        density = (avg_num_veh_on / segment_length) * 1000

        density_out = density
            
        print('density', round(density,3), 'veh/km ', end='; ')

        flow = (veh_entered_sum / aggregation_time) * 3600
        # the point_loop should act as point detector
        # not expected to have one single car on it for more than 1 time step
        print('flow', flow, 'veh/h', end='; ')
        
        # one speed metric is enough
        # this equals to spot speed as induction loop is short
        mean_speed = (mean_speed_on_sum / aggregation_time) * 3.6
        print('speed', round(mean_speed,2), 'km/h', end='; ')

        # before
        avg_num_veh_on = veh_on_sum_b / aggregation_time
        density = avg_num_veh_on / segment_length

        density_in = density
            
        print('density', round(density,3), 'veh/m ', end='; ')

        flow = veh_entered_sum_b / aggregation_time 
        # the point_loop should act as point detector
        # not expected to have one single car on it for more than 1 time step
        print('flow', flow, 'veh/s', end='; ')
        
        # one speed metric is enough
        # this equals to spot speed as induction loop is short
        mean_speed = mean_speed_on_sum_b / aggregation_time
        print('speed', round(mean_speed,2), 'm/s')

        occupancy = occupancy_sum / aggregation_time
        print('OCC', round(occupancy, 2))
        occ.append(occupancy)
        flw.append(flow)
        sp = traci.lane.getMaxSpeed(seg_0_before_top)
        #traci.lane.setMaxSpeed(seg_0_before_top, sp - 0.1)

        if step == 10000:
            print('OCC', occ)
            print('FLW', flw)
            plt.scatter(occ, flw)
            plt.show()

        print('NUM', round(num_sum / aggregation_time, 2))
        
        # adapt the speed limit to make traffic fluent
        
        occupancy_desired = 10 # %
        occupancy_old = occupancy
        K_r = 25 # mu veh/h/%
        flow_old = flow

        flow_new = flow_old + K_r * (occupancy_desired - occupancy_old)
        speed_old_top = traci.lane.getMaxSpeed(seg_0_before_top)
        speed_old_bottom = traci.lane.getMaxSpeed(seg_0_before_bottom)
        

        if flow_new > flow_old:
            speed_new_top = speed_old_top - 0.1
            speed_new_bottom = speed_old_bottom - 0.1
        else:
            speed_new_top = speed_old_top + 0.1
            speed_new_bottom = speed_old_bottom + 0.1

        print('SPEED', speed_new_top)
        traci.lane.setMaxSpeed(seg_0_before_top, speed_new_top)
        traci.lane.setMaxSpeed(seg_0_before_bottom, speed_new_bottom)
        
        # plot speed to flow

        """
        if density_out < density_in:
            top = traci.lane.getMaxSpeed(seg_0_before_top)
            bottom = traci.lane.getMaxSpeed(seg_0_before_bottom)

            traci.lane.setMaxSpeed(seg_0_before_top, top + 0.1)
            traci.lane.setMaxSpeed(seg_0_before_bottom, bottom + 0.1)
        """

        # reset accum
        occupancy_sum = 0
        num_sum = 0

        veh_on_sum = 0
        veh_entered_sum = 0
        mean_speed_on_sum = 0

        veh_on_sum_b = 0
        veh_entered_sum_b = 0
        mean_speed_on_sum_b = 0


        

traci.close()