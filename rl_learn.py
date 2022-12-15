from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
from statistics import mean
import traci
from matplotlib import pyplot as plt

from rl_utilities.model import *
from rl_gym_environments import *

# ----------------------------------------------- RL ENVIROMENT START -----------------------------------------------

env = SUMOEnv()
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

