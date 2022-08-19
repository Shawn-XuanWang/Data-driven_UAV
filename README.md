# UAV Data-driven Control Environment
This is the test environement for [IEEE ICMLA 2022 UAV Data-driven Control Challenge](https://www.icmla-conference.org/icmla22/Challenge_ICMLA.pdf). 
## Overview
This challenge consists of two tasks:
- Safe UAV control in a risky environment.
- Multi-UAV cooperative control in a risky environment.

For the first task, one needs to design a control policy for an autonomous UAV (in 3-D space but with unknown dynamics) to fly through an unstructured environment from a distribution of initial positions to a fixed target position (their coordinates are given as prior knowledge), with avoidance of some risky areas in the environment. The overall score of the task will be evaluated as a combination of (i) the closeness of the UAV’s final position to the target location and (ii) the UAV’s
violation of risky areas throughout its motion.

For the second task, one needs to design control policies for multiple UAVs, with each UAV completing a similar task as the first task. An additional consideration in this task is the collision avoidance among all UAVs, i.e., the distance between any two UAVs must be greater than the sum of their safe radii. If this constraint is violated at any time instance, the overall score of the system drops to zero.


## The Simulated Environment
### Getting started 
Download the source code folder and establish new projects for Single_UAV and Multi-UAV, respectively. Make sure all associated files are properly linked to the project. For Pycharm (recommended), simply use each folder to open a new project.

Requirements:
- Python  3.10 tested
- numpy  1.23.2 tested 
- casadi  3.5.5.post2 tested
- matplotlib  3.5.3 tested

### The environment
***Do Not Modify envS.py or envM.py***

The environment simulates the full dynamics of a quadrotor with a 13-dimension state space (3 positions+4 quaternions + 3 velocities + 3 angular velocities) and a 4-dimension input space (4 thrust forces generalized by 4 propellers). The inputs are clipped before execution.
The following interface functionalities are offered by the environment:
- Set(state_val): set the state of a UAV to an arbitrary state_val.
- Get(): get the current state of a UAV.
- Step(input): simulate the dynamics of a UAV for one-step forward given input (Note the
input will be clipped before simulation)
- Reset(): reset the state of a UAV to an initial value (or a sample from the initial distribution)
- RiskEval(): if the current state of a UAV is outside the risky areas, it will return 0, otherwise,
it will return a positive value depending on how much the UAV enters the risky areas.
- Render(): visualize the environment/task.

In each folder, `example_script.py` provides examples to execute the mentioned interface functionalities to retrieve information from the environment.


## Evaluation
### Self evaluation
For each task, `test_random.py` evaluates the designed controller under a randomly generated environment. The control time horizon `T` can be set as an arbitrary positive value, depending on how long it takes the controller to drive the UAV to the target location. The running costs and terminal costs depend on the controller energy consumption and the closeness of the UAVs to the target. 

Determine your time horizon `T` and design control sequence `u_traj` to obtain a maximim `Final reward`.

If the UAVs crash into an obstacle or crash with each other, the reward is set to `-Inf`

### Final evaluation 
For each task, `text_eval_1.py` is one of the __3__ environments that we will finally use to evaluate the designed controller. (Another 2 environments will be used for final evaluation but are kept confidential to the participants.) The control time horizon `T` can be set as an arbitrary positive value, depending on how long it takes the controller to drive the UAV to the target location. The running costs and terminal costs depend on the controller energy consumption and the closeness of the UAVs to the target. 

Determine your time horizon `T` and design control sequence `u_traj` to obtain a maximim `Final reward`.

If the UAVs crash into an obstacle or crash with each other, the reward is set to `-Inf`

## Questions
All questions shoudld be forwarded to Xuan Wang [xwang64@gmu.edu](xwang64@gmu.edu) or Wanxin Jin [wanxinjin@gmail.com](wanxinjin@gmail.com)
