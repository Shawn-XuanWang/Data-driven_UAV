from casadi import *
from envM import Quadrotor_swarm

num_uav = 4
UAV_team = Quadrotor_swarm(num_uav)
UAV_team.setObstacles([[0, 0, 7], [2, 4, 3], [4, 1, 9], [-3.5, 2, 7], [-2.5, -3.5, 5.5], [-2, 2, 10], [-3, -1, 4]], 1.2)
Target_state = [np.array([5, 5, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]), np.array([6, 5, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]),
                np.array([5, 6, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]), np.array([6, 6, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])]
# Do not change Target_state
dt = 0.05
# Do not change dt

T = 100
# Time horizon, can be set as arbitrary positive value.
# Depends on how long it takes the controller to drive the UAV to the target location
# [5, 5, 12], [6 ,5 ,12], [5, 6, 12], [6, 6, 12]

# Set initial states
init_team = [np.array([-4.9, -5.1, 2.0, 0, 0, 0, 0.6, 0.8, 0, 0, 0, 0, 0]),
             np.array([-5.9, -5.1, 2.1, 0, 0, 0, 0.6, 0.8, 0, 0, 0, 0, 0]),
             np.array([-4.9, -6.0, 1.9, 0, 0, 0, 0.6, 0.8, 0, 0, 0, 0, 0]),
             np.array([-6.0, -6.1, 2.0, 0, 0, 0, 0.6, 0.8, 0, 0, 0, 0, 0])]

UAV_team.setState(init_team)  # Set initial states

# This should be replaced by the designed control input sequence
u_traj = [[np.random.randn(4) for _ in range(num_uav)] for _ in range(T)]

# This simulates the UAV control and returns the final reward. Do not change the following parts
Total_reward = 0
for ut in u_traj:
    for i in range(num_uav):
        UAV_team.step(ut)
        current_risk = UAV_team.RiskEval()
        if sum(current_risk[0]) > 0 or sum(current_risk[1]) > 0:  # obstacle violation.
            Total_reward = float('-inf')
        Total_reward = Total_reward - np.linalg.norm(ut) ** 2 - np.linalg.norm(
            numpy.array(UAV_team.getState()) - numpy.array(Target_state))  # Running cost.
        UAV_team.render(dt * 2)  # 2 times slow motion

Total_reward = Total_reward - 100 * np.linalg.norm(
    numpy.array(UAV_team.getState()) - numpy.array(Target_state))  # Terminal cost.
print('Final Reward:')
print(Total_reward)
