from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random
import os, sys
from statistics import mean
import traci
from matplotlib import pyplot as plt
import control_mechanisms

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

import rl.rl_occ_control as rl_occ_control
import rl_utils

env = rl_occ_control.SUMOEnv()
actions = env.action_space.n
states = env.observation_space.shape[0]
model = rl_utils.build_model(states, actions)
dqn = rl_utils.build_agent(model, actions)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

dqn.load_weights('dqn_weights.h5f')

_ = dqn.test(env, nb_episodes=5, visualize=True)