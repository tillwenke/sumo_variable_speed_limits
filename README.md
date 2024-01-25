# SUMO Traffic Simulation

Project repository for Intelligent Transportation Systems course in Fall 2022 at University of Tartu.
You can find a 3 to 2 lane merge scenario on a highway and differnent approaches to resolve emerging congestions including *reinforcment learning* and classical *rule-based* approaches.
In order to learn about our outcome you can read "ITS_Report-VSL.pdf".

# Getting started
Embed the cloned project in a [PyVenv](https://docs.python.org/3/library/venv.html) (or conda env if you prefer) and install the requiremnts listed in requirements.txt

And get yourself the [SUMO traffic simulation](https://www.eclipse.org/sumo/).

# Launching the different approaches

New speed limits are calculated/ set every 30 s in order to change it change *aggregation_time* in the respective files.

Differnt traditional algorithmic approaches can be found in *control_algotihms.py* which can be applied to the scenario by setting `approach=` in *variable_speed_environment.py* and running it.

A try to solve the congestion problem with reinforcement learning can be found in the custom OpenAI Gym-environment in *rl_utils/rl_gym_environments*. Models can be trained using *rl_learn.py* - pre-trained models are provided under *rl_models/*. Their outcome can be assessed running *rl_test.py*.

# Investigation outcomes via metrics

Unfortunatelly no algorithm convincingly resolved the congestion so eye-test cannot be used to assess their achievements. Run algorithmns store their metrics in *metrics/* - most importantly the mean speed of cars over the whole stretch of the road. To combine the metrics of differnet approaches visually run *metrics/combine.py*.

## Credits

To Ellen Grumert - partially you can find implementations in the SUMO simulation that have been described in  [her work](http://vti.diva-portal.org/smash/get/diva2:794891/FULLTEXT01.pdf).
