# inspired by https://github.com/nicknochnack/OpenAI-Reinforcement-Learning-with-Custom-Environment

# Inspired by OpenAI Gym for constructing a custom environment.
from gym import Env
from gym.spaces import Discrete, Box
# Utilized for numerical operations and array manipulations.
import numpy as np
# Used for statistical calculations like averaging.
from statistics import mean
# Allows interaction with the SUMO traffic simulator for real-time data manipulation.
import traci
# For creating and displaying plots and graphs to visualize data.
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
        
        # Calculate reward based on the current state, which is influenced by the vehicle's occupancy and speed.
        # This function evaluates how well the current state aligns with the desired state, guiding the agent's learning.
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
        # The mean_edge_speed array is initialized to track the average speed of vehicles on each edge of the road network.
        # This metric is crucial for understanding traffic flow and congestion levels.
        
        # Vehicle time sum and occupancy sum are accumulators for calculating traffic flow and occupancy, respectively.
        # These metrics are essential for assessing the efficiency of traffic management strategies.
        
        # Looping through the aggregation time allows for the collection of traffic data over a specified interval.
        # This approach helps in smoothing out the variability in traffic metrics, providing a more stable state representation.
        
        # Emissions are tracked to assess the environmental impact of traffic. Lower emissions indicate more efficient traffic flow.
        
        # The maximum occupancy in front of the merge area is determined by averaging the occupancy across several sensors.
        # This metric helps in understanding the density of traffic approaching the merge, which is critical for managing congestion.
                cvs_sum += lane_stdv / lane_avg
            cvs_seg = cvs_sum / len(seg)
            if np.isnan(cvs_seg):
                cvs_seg = 0
            self.cvs_seg_time[i].append(cvs_seg)

        # collected metrics are devided by the aggregation time to get the average values
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
        # The CVS (Coefficient of Variation of Speed) is calculated for each road segment to monitor traffic safety.
        # A higher CVS indicates greater variability in speed, which can be a sign of unsafe driving conditions.
        
        # Aggregating metrics over time provides average values that offer a clearer picture of traffic conditions.
        # This approach reduces the impact of transient spikes in data, leading to more reliable state representations and reward calculations.
        
        # The mean road speed is calculated as an average of the mean speeds across all edges, providing a comprehensive view of traffic flow.
        # This metric is crucial for the reinforcement learning model to understand the overall efficiency of the traffic network.
        
        # Traffic flow after the merge is calculated to assess the effectiveness of traffic management strategies in that area.
        # High flow rates indicate smooth traffic movement, while low rates may suggest congestion or inefficiencies.
        
        # The occupancy metric provides insight into the level of road usage, which is essential for congestion management and traffic optimization.

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
    
        # Normalizing the mean road speed from m/s to km/h makes the speed representation more intuitive and comparable to real-world scenarios.
        # This normalization is crucial for the learning process as it ensures that the speed values are within a range that the model can effectively interpret and act upon.