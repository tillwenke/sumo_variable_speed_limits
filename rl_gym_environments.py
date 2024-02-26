# This file defines the SUMOEnv class, which is a custom environment for reinforcement learning in traffic management simulations.
# The environment is built on top of the OpenAI Gym interface, allowing for the integration of various reinforcement learning models.
# It interfaces with the Simulation of Urban MObility (SUMO) through the "traci" interface to simulate traffic scenarios and control traffic lights or speed limits.
# This environment is crucial for setting up and evaluating reinforcement learning models aimed at optimizing traffic flow and reducing congestion.
# inspired by https://github.com/nicknochnack/OpenAI-Reinforcement-Learning-with-Custom-Environment

from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
from statistics import mean
import traci
from matplotlib import pyplot as plt

from rl_utilities.reward_functions import *
from rl_utilities.model import *
from simulation_utilities.road import *
from simulation_utilities.setup import *

class SUMOEnv(Env):
    def __init__(self):
        # Actions we can take, down, stay, up
        self.action_space = Discrete(3)
        # avg speed array
        self.observation_space = Box(low=np.array([0]), high=np.array([1]))
        self.state = 0 # occupancy
        self.speed_limit = 120 # to be changed actively
        # Set simulation length
        self.aggregation_time = 30
        self.sim_length = 3600/self.aggregation_time # 30 x 120 = 3600 steps        

        # SUMO specific
        #run sumo simulation
        traci.start(sumoCmd)

        #self.reward_func = scipy.stats.norm(105, 7.5).pdf
        self.reward_func = quad_occ_reward

        self.mean_speeds = []
        self.flows = []

        self.emissions_over_time = []

        self.cvs_seg_time = []
        for i in range(len(all_segments)):
            self.cvs_seg_time.append([])
        
    def step(self, action):
        # Apply action
        # 0 -1 = -1 
        # 1 -1 = 0 
        # 2 -1 = 1 
        self.speed_limit += (action - 1) * 10
      
        # only cap the speed using lower bounds to let the algorithm learn itself
        if self.speed_limit < 10:
            self.speed_limit = 10       
        

        # copied from mtfc
        speed_new = self.speed_limit / 3.6 # km/h to m/s
        road_segments = [seg_1_before]

        for segment in road_segments:
           [traci.lane.setMaxSpeed(lane, speed_new) for lane in segment]

        # Reduce simulation length by 1 second
        self.sim_length -= 1 
        
        # Calculate reward
        reward = self.reward_func(self.state)
        
        # Check if shower is done
        if self.sim_length <= 0: 
            done = True
        else:
            done = False

        # ------------------------------ SUMO ------------------------------

        # the only relevant parameter until now
        mean_edge_speed = np.zeros(len(edges))

        # set accumulators
        veh_time_sum = 0        

        # simulate one step in SUMO to get new state
        occupancy_sum = 0

        for i in range(self.aggregation_time):
            traci.simulationStep() 
                    
            # GATHER METRICS FROM SENSORS    
            # for some it is important to average over the number of lanes on the edge

            # AFTER
            veh_time_sum += sum([traci.inductionloop.getLastStepVehicleNumber(loop) for loop in loops_after])

            emission_sum = 0
            for i, edge in enumerate(edges):
                mean_edge_speed[i] += traci.edge.getLastStepMeanSpeed(edge)
                emission_sum += traci.edge.getCO2Emission(edge)
            self.emissions_over_time.append(emission_sum)

            # BEFORE

            # collecting the number of vehicles and occupancy right in front (or even in) of the merge area
            # choose max occupancy of a few sensors            
        # Vehicle time sum and emissions are crucial metrics as they directly relate to the efficiency of traffic flow and the environmental impact of the traffic system. Lower vehicle times and emissions indicate a more efficient and environmentally friendly traffic management system, which are key objectives for the reinforcement learning model to achieve.
            occ_max = 0
            for loops in loops_before:
                occ_loop = sum([traci.inductionloop.getLastStepOccupancy(loop) for loop in loops]) / len(loops)
                if occ_loop > occ_max:
                    occ_max = occ_loop

            occupancy_sum += occ_max
        
        # monitor the safety of road segments (CVS) - stores cvs value for each segment for each aggregation time step
        for i, seg in enumerate(all_segments):
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
        # Aggregating metrics over a defined time period allows for a more stable and representative measurement of the traffic conditions and environmental impact. This approach smooths out the variability in individual measurements, providing a clearer picture of the overall system performance for the reinforcement learning model to learn from.
                cvs_sum += lane_stdv / lane_avg
            cvs_seg = cvs_sum / len(seg)
            if np.isnan(cvs_seg):
                cvs_seg = 0
            self.cvs_seg_time[i].append(cvs_seg)

        # collected metrics are devided by the aggregation time to get the average values
        # Choosing the maximum occupancy from sensors before the merge area is significant as it reflects the highest level of congestion that vehicles experience approaching the merge. This metric helps the reinforcement learning model to prioritize actions that alleviate congestion at critical points, improving overall traffic flow.
        # OVERALL
        mean_edge_speed = mean_edge_speed / self.aggregation_time # first is acutally a sum
        mean_road_speed = sum(mean_edge_speed) / len(mean_edge_speed)
        self.mean_speeds.append(mean_road_speed) 

        # AFTER THE MERGE
        flow = (veh_time_sum / self.aggregation_time) * 3600
        self.flows.append(flow)     

        occupancy = occupancy_sum / self.aggregation_time      

        # ------------------------------ SUMO ------------------------------
        
        # gets the avg speed from the simulation
        # normalize the speed
        self.state = occupancy
        self.state_speed = mean_road_speed * 3.6  # m/s to km/h

        # Set placeholder for info
        info = {}
        
        # Return step information
        return self.state, reward, done, info

    def render(self):
        # Implement viz
        pass
    
    def reset(self):
        self.mean_speeds = []

        # Reset params
        self.state = 0
        self.state_speed = 0
        self.speed_limit = 120
        # Reset time
        self.sim_length = 3600/self.aggregation_time

        # Reset SUMO
        traci.close(False)
        traci.start(sumoCmd)

        return self.state
    