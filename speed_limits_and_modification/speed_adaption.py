import os, sys
import traci

#setup
if 'SUMO_HOME' in os.environ:
     tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
     sys.path.append(tools)
else:
     sys.exit("please declare environment variable 'SUMO_HOME'")


#init sumo simulation
sumoBinary = "/usr/bin/sumo-gui"
delay = '0.1'
sumoCmd = [sumoBinary, "-c", "different_speed_on_edges-on_ramp.sumocfg", "--step-length", delay, '--start'] #no need to specify path to sumocfg

#run sumo simulation
traci.start(sumoCmd)

# parameters and sumo objects
cycle_time = 100
loop_id = "loop_mid"
lane_1_id = "E9_0"
lane_2_id = "E9_1"
low_speed = 1
high_speed = 10


step = 0
speed_sum = 0

# run till all cars are gone
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
 
    step += 1

    # here speed - could also use getLastStepVehicleNumber as density measurement
    # need to think about how to retrieve the real metrics we want
    # induction loops in sumo set up in additional files and similar to those in reality
    current_speed = traci.inductionloop.getLastStepMeanSpeed(loop_id)
    if current_speed >= 0:
        speed_sum += current_speed
    
    # adjust the speed of the edge infront of junction every "cycle_time" steps
    if step % cycle_time == 0:
        speed_avg = speed_sum / cycle_time
        speed_sum = 0

        if speed_avg > 2:
            traci.lane.setMaxSpeed(lane_1_id, high_speed)
            traci.lane.setMaxSpeed(lane_2_id, high_speed)
        else:
            traci.lane.setMaxSpeed(lane_1_id, low_speed)
            traci.lane.setMaxSpeed(lane_2_id, low_speed)


        print(speed_avg, traci.inductionloop.getSubscriptionResults(loop_id), traci.inductionloop.getLastStepMeanSpeed(loop_id))

traci.close()