import os
import sys
import traci
import math
import random

# Changing flow values during simulation:
# https://www.eclipse.org/lists/sumo-user/msg05014.html
# https://www.eclipse.org/lists/sumo-user/msg10414.html
# https://sumo.dlr.de/pydoc/traci._calibrator.html#CalibratorDomain-setFlow
# You cannot change values of flow during simulation. only way to adjust 
# the flow is by spawning cars manually.

def spawn_cars(step, probability):
    if random.random() < probability:
        traci.vehicle.add(str(step), 'highway', 'normal_car', departLane='random')

# Change path to SUMO tools path.
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

delay = 50
step = 0

# Run SUMO simulation.
sumo_binary = 'C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo-gui.exe'
sumo_cmd = [sumo_binary, "-c", "config.sumocfg", '--start', '--delay', str(delay)]
traci.start(sumo_cmd)

while step < 5000:

    # Change probability of spawning a car
    # with sinus function.
    amplitude = 0.5
    frequency = 0.05
    prob = min(math.sin(step*frequency) * amplitude + 0.5, 0.8)

    print(f'step: {step} probability: {prob:.2f}')
    spawn_cars(step, prob)

    traci.simulationStep()
    step += 1

traci.close()