from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random
import os, sys
from statistics import mean
import traci
from matplotlib import pyplot as plt
import control_mechanisms
import pandas as pd

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

import gym_envs
import rl_utils

env = gym_envs.occ_SUMOEnv()
actions = env.action_space.n
states = env.observation_space.shape
model = rl_utils.build_model(states, actions)
dqn = rl_utils.build_agent(model, actions)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

dqn.load_weights('dqn_occ_weights.h5f')

_ = dqn.test(env, nb_episodes=1, visualize=False)

# Save metrics into csv file.
approach = 'rl'
with open(f'./metrics/{approach}_metrics.csv', 'w+') as metrics_file:
    list_to_string = lambda x: ','.join([ str(elem) for elem in x ]) + '\n'
    metrics_file.write(list_to_string(env.mean_speeds))
    metrics_file.write(list_to_string(env.flows))
    metrics_file.write(list_to_string(env.emissions_over_time))

pd.DataFrame(env.cvs_seg_time).to_csv(f'./metrics/{approach}_cvs.csv', index=False, header=False)

plt.plot(env.mean_speeds)
plt.show()