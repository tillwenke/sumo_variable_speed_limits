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

import gym_envs
import rl_utils

env = gym_envs.speed_SUMOEnv()
actions = env.action_space.n
states = env.observation_space.shape
model = rl_utils.build_model(states, actions)
dqn = rl_utils.build_agent(model, actions)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

dqn.load_weights('dqn_weights.h5f')

_ = dqn.test(env, nb_episodes=1, visualize=False)

plt.plot(env.mean_speeds)
plt.show()