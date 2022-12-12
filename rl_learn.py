from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random
import os, sys
from statistics import mean
import traci
from matplotlib import pyplot as plt
import control_algorithms
import scipy.stats

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

from rl_utilities.model import *
import gym_envs


# ----------------------------------------------- RL ENVIROMENT START -----------------------------------------------

env = gym_envs.SUMOEnv()
print(env.observation_space)
print(env.observation_space.sample(), env.action_space.sample())

# ----------------------------------------------- RL NOW WITH MODEL -----------------------------------------------

states = env.observation_space.shape
actions = env.action_space.n
print(states)
model = build_model(states, actions)
print(model.summary())

dqn = build_agent(model, actions)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])
dqn.fit(env, nb_steps=12000, visualize=False, verbose=1, log_interval=12000)

plt.plot(env.mean_speeds)
plt.show()

dqn.save_weights('models/model_name.h5f', overwrite=True)

scores = dqn.test(env, nb_episodes=1, visualize=False)
print(np.mean(scores.history['episode_reward']))

#finally
traci.close(False)

