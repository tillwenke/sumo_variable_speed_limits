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
import gym_envs


# ----------------------------------------------- RL ENVIROMENT START -----------------------------------------------

env = gym_envs.occ_SUMOEnv()
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
dqn.fit(env, nb_steps=12000, visualize=False, verbose=1, log_interval=12000)

dqn.save_weights('dqn_occ_weights.h5f', overwrite=True)

scores = dqn.test(env, nb_episodes=1000, visualize=False)
print(np.mean(scores.history['episode_reward']))




#finally
traci.close(False)

