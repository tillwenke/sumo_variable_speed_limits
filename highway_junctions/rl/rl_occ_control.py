from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random
import os, sys
from statistics import mean
import traci
from matplotlib import pyplot as plt
import control_mechanisms
import scipy.stats

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

import rl_utils

max_speed = 150

class SUMOEnv(Env):
    def __init__(self):
        # Actions we can take, down, stay, up
        self.action_space = Discrete(3)
        # avg speed array
        self.observation_space = Box(low=np.array([0]), high=np.array([1]))
        self.state = 0 # occupancy
        self.state_speed = 0
        self.speed_limit = 120 # to be changed actively
        # Set simulation length
        self.sim_length = 120 # 30 x 120 = 3600 steps
        # SUMO specific
        #run sumo simulation
        traci.start(sumoCmd)

        #get 11% occ as best
        self.reward_func = scipy.stats.norm(0.11, 0.01).pdf
        
    def step(self, action):
        # Apply action
        # 0 -1 = -1 
        # 1 -1 = 0 
        # 2 -1 = 1 
        self.speed_limit += (action - 1) * 10
        if self.speed_limit > max_speed:
            self.speed_limit = max_speed
        if self.speed_limit < 50:
            self.speed_limit = 50        

        # copied from mtfc
        speed_new = self.speed_limit / 3.6 # km/h to m/s
        road_segments = [seg_1_before]

        for segment in road_segments:
            [traci.lane.setMaxSpeed(lane, speed_new) for lane in segment]

        # Reduce simulation length by 1 second
        self.sim_length -= 1 
        
        # Calculate reward
        # TODO define desired values
        # only depend on the mean speed not the occupancy [1]
        # account for normalized mean speed

        # binary reward doesnt perform well
        """
        if self.state_speed >=(90/max_speed) and self.state_speed <= (120/max_speed):
            reward = 1 
        else: 
            reward = -1 
        """

        # let max reward be 1
        reward = self.reward_func(self.state)/ 39.89422804014327
        
        # Check if shower is done
        if self.sim_length <= 0: 
            done = True
        else:
            done = False

        # ------------------------------ SUMO ------------------------------

        # the only relevant parameter until now
        mean_edge_speed = np.zeros(len(edges))

        # simulate one step in SUMO to get new state
        aggregation_time = 30

        for i in range(aggregation_time):
            traci.simulationStep() 
                    
            # GATHER METRICS FROM SENSORS    
            # for some it is important to average over the number of lanes on the edge

            # AFTER
            #veh_space_sum += sum([traci.lanearea.getLastStepVehicleNumber(detector) for detector in detectors_after])
            #veh_time_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_after])

            #mean_speed = sum([traci.inductionloop.getLastStepMeanSpeed(loop) for loop in loops_after]) / len(loops_after)
            # speed -1 indicated no vehicle on the loop
            #if mean_speed >= 0:
                #mean_speed_sum += mean_speed

            for i, edge in enumerate(edges):
                mean_edge_speed[i] += traci.edge.getLastStepMeanSpeed(edge)
                #emissions[i] += traci.edge.getCO2Emission(edge)
            

            # BEFORE
            #veh_space_before_sum += sum([traci.lanearea.getLastStepVehicleNumber(detector) for detector in detectors_before])

            # collecting the number of vehicles and occupancy right in front (or even in) of the merge area
            # choose max occupancy of a few sensors
            occupancy_sum = 0
            occ_max = 0
            for loops in loops_before:
                occ_loop = sum([traci.inductionloop.getLastStepOccupancy(loop) for loop in loops]) / len(loops)
                if occ_loop > occ_max:
                    occ_max = occ_loop

            occupancy_sum += occ_max
            #num_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_before[0]]) # only at one sensor


        # collected metrics are devided by the aggregation time to get the average values
        # OVERALL
        mean_edge_speed = mean_edge_speed / aggregation_time # first is acutally a sum
        #print(mean_edge_speed)
        mean_road_speed = sum(mean_edge_speed) / len(mean_edge_speed)
        #print(mean_road_speed)
        #ms.append(mean_road_speed)

        # AFTER THE MERGE
        #density = ((veh_space_sum / aggregation_time) / detector_length) * 1000
        #flow = (veh_time_sum / aggregation_time) * 3600
        #flw.append(flow)
        #mean_speed = (mean_speed_sum / aggregation_time) * 3.6 # one speed metric is enough - equals to spot speed

        # BEFORE THE MERGE
        #density_before = ((veh_space_before_sum / aggregation_time) / detector_length) * 1000
        #dens.append(density_before)

        occupancy = occupancy_sum / aggregation_time
        #occ.append(occupancy)
        #num = num_sum / aggregation_time
        
        # reset accumulator
        #veh_time_sum = 0
        #veh_space_sum = 0
        #mean_speed_sum = 0

        #mean_edge_speed = np.zeros(len(edges))
        #mean_road_speed = 0

        #veh_space_before_sum = 0
        #occupancy_sum = 0
        #num_sum = 0

        # ------------------------------ SUMO ------------------------------
        
        # gets the avg speed from the simulation
        # normalize the speed
        self.state = occupancy
        self.state_speed = mean_road_speed * 3.6  # m/s to km/h
        print('>', action, self.state, reward, self.speed_limit)

        # Set placeholder for info
        info = {}
        
        # Return step information
        return self.state, reward, done, info

    def render(self):
        # Implement viz
        pass
    
    def reset(self):
        # Reset params
        self.state = 0
        self.state_speed = 0
        self.speed_limit = 120
        # Reset time
        self.sim_length = 120

        # Reset SUMO
        traci.close(False)
        traci.start(sumoCmd)

        return self.state
    


# from https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html

#setup
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


#init sumo simulation
executable = 'sumo-gui.exe' if os.name == 'nt' else 'sumo-gui'
sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin', executable)
sumoCmd = [sumoBinary, "-c", "2_1_merge.sumocfg", '--start', '--quit-on-end']


# ----------------------------------------------- VARIABLE SETTING -----------------------------------------------

edges = ["seg_10_before","seg_9_before","seg_8_before","seg_7_before","seg_6_before","seg_5_before","seg_4_before","seg_3_before","seg_2_before","seg_1_before","seg_0_before","seg_0_after","seg_1_after"]

# LANES & MAXIMUM SPEEDS
# naming:
# laneSEGment_"number, 0 means closest to merge zone"_"before or after the merge"_"lane number counting from bottom to top"
seg_10_before = ["seg_10_before_2", "seg_10_before_1", "seg_10_before_0"]
seg_9_before = ["seg_9_before_2", "seg_9_before_1", "seg_9_before_0"]
seg_8_before = ["seg_8_before_2", "seg_8_before_1", "seg_8_before_0"]
seg_7_before = ["seg_7_before_2", "seg_7_before_1", "seg_7_before_0"]
seg_6_before = ["seg_0_before_2", "seg_6_before_1", "seg_6_before_0"]
seg_5_before = ["seg_5_before_2", "seg_5_before_1", "seg_5_before_0"]
seg_4_before = ["seg_4_before_2", "seg_4_before_1", "seg_4_before_0"]
seg_3_before = ["seg_3_before_2", "seg_3_before_1", "seg_3_before_0"]
seg_2_before = ["seg_2_before_2", "seg_2_before_1", "seg_2_before_0"]
seg_1_before = ["seg_1_before_2", "seg_1_before_1", "seg_1_before_0"]
seg_0_before = ["seg_0_before_2", "seg_0_before_1", "seg_0_before_0"]
segments_before = [seg_10_before, seg_9_before, seg_8_before, seg_7_before, seg_6_before, seg_5_before, seg_4_before, seg_3_before, seg_2_before, seg_1_before, seg_0_before]

seg_0_after = ["seg_0_after_1", "seg_0_after_0"]
seg_1_after = ["seg_1_after_1", "seg_1_after_0"]
segments_after = [seg_0_after, seg_1_after]

low_speed = 15 # 50 km/h
speed_max = 33.33 # 120 km/h

# ROAD SENSORS / INDUCTION LOOPS
# defined in additional.xml
# naming:
# equal to lanes
# keeping to the sumo objects used
# loop = induction loop ... for measurements at a point
# detector = lane are detector ... for measurements along a lane

loops_beforeA = ["loop_seg_0_before_2A", "loop_seg_0_before_1A", "loop_seg_0_before_0A"]
loops_beforeB = ["loop_seg_0_before_2B", "loop_seg_0_before_1B", "loop_seg_0_before_0B"]
loops_beforeC = ["loop_seg_0_before_2C", "loop_seg_0_before_1C", "loop_seg_0_before_0C"]
loops_beforeD = ["loop_seg_0_before_2D", "loop_seg_0_before_1D", "loop_seg_0_before_0D"]
loops_before = [loops_beforeA, loops_beforeB, loops_beforeC, loops_beforeD]
detectors_before = ["detector_seg_0_before_2", "detector_seg_0_before_1", "detector_seg_0_before_0"]

loops_after = ["loop_seg_0_after_1", "loop_seg_0_after_0"]
detectors_after = ["detector_seg_0_after_1", "detector_seg_0_after_0"]

detector_length = 50 #meters

# ----------------------------------------------- RL ENVIROMENT START -----------------------------------------------

env = SUMOEnv()
print(env.observation_space)
print(env.observation_space.sample(), env.action_space.sample())
""""
obs = env.reset()
print("The initial observation is {}".format(obs))

# Sample a random action from the entire action space
random_action = env.action_space.sample()

# # Take the action and get the new observation space
new_obs, reward, done, info = env.step(random_action)

# # Take the action and get the new observation space
new_obs, reward, done, info = env.step(random_action)
print("The new observation is {}".format(new_obs))

episodes = 3
for episode in range(1, episodes+1):
    state = env.reset()
    done = False
    score = 0 
    
    while not done:
        #env.render()
        action = env.action_space.sample()
        n_state, reward, done, info = env.step(action)
        score+=reward
    print('Episode:{} Score:{}'.format(episode, score))
"""

    # ----------------------------------------------- RL NOW WITH MODEL -----------------------------------------------

states = env.observation_space.shape
actions = env.action_space.n

model = rl_utils.build_model(states, actions)
print(model.summary())

dqn = rl_utils.build_agent(model, actions)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])
dqn.fit(env, nb_steps=200000, visualize=False, verbose=1, log_interval=40000)

dqn.save_weights('dqn_weights.h5f', overwrite=True)

scores = dqn.test(env, nb_episodes=1000, visualize=False)
print(np.mean(scores.history['episode_reward']))




#finally
traci.close(False)

