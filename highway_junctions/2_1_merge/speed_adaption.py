import os, sys
import traci


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
loop_before_top = "loop_seg_0_before_1"
loop_before_top = "loop_seg_0_before_0"
loop_after = "loop_seg_0_after_0"

seg_0_before_top = "seg_0_before_0"
seg_0_before_bottom = "seg_0_before_1"
seg_0_after = "seg_0_after_0"

# define metrics
density = 0
veh_sum = 0

flow = 0
time_mean_speed = 0
space_mean_speed = 0


low_speed = 1
high_speed = 10


step = 0
speed_sum = 0

aggregation_time = 100 # always aggregate the last 100 step to make decision in the present

# run till all cars are gone
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep() 
    step += 1

    this_num_veh =  traci.inductionloop.getLastStepVehicleNumber(loop_after)
    

    veh_sum += this_num_veh

    this_step_speed = traci.inductionloop.getLastStepMeanSpeed(loop_after)
    if this_step_speed >= 0:
        speed_sum += this_step_speed


    if step < 100:
        print(step-1, this_num_veh, '|', this_step_speed)
    
    if step % aggregation_time == 0:

        print(veh_sum, '/', aggregation_time, 'steps')
        veh_sum = 0

        """
        speed_avg = speed_sum / cycle_time
        speed_sum = 0

        if speed_avg > 2:
            traci.lane.setMaxSpeed(lane_1_id, high_speed)
            traci.lane.setMaxSpeed(lane_2_id, high_speed)
        else:
            traci.lane.setMaxSpeed(lane_1_id, low_speed)
            traci.lane.setMaxSpeed(lane_2_id, low_speed)

        print(speed_avg, traci.inductionloop.getSubscriptionResults(loop_id), traci.inductionloop.getLastStepMeanSpeed(loop_id))
        """


traci.close()