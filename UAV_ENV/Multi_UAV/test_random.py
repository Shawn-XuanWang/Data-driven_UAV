from casadi import *
from envM import Quadrotor_swarm

num_uav = 4
UAV_team = Quadrotor_swarm(num_uav)
UAV_team.setRandomObstacles(n_obstacle=7, ball_r=1.2)
Target_state = [np.array([5, 5, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]), np.array([6, 5, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]),
                np.array([5, 6, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]), np.array([6, 6, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])]
dt = 0.05

T = 100
# Time horizon, can be set as arbitrary positive value.
# Depends on how long it takes the controller to drive the UAV to the target location [5, 5, 12]

# Set initial states
init_team = [np.random.randn(UAV_team.n_state) for _ in range(num_uav)] # UAV_team.n_state = 13
init_team[0][0:3] = np.array([-5, -5, 2] + 0.5*np.random.randn(3))  # Set position of UAV0 close to [-5, -5, 2]
init_team[1][0:3] = np.array([-6, -5, 2] + 0.5*np.random.randn(3))  # Set position of UAV1 close to [-6, -5, 2]
init_team[2][0:3] = np.array([-5, -6, 2] + 0.5*np.random.randn(3))  # Set position of UAV2 close to [-5, -6, 2]
init_team[3][0:3] = np.array([-6, -6, 2] + 0.5*np.random.randn(3))  # Set position of UAV3 close to [-6, -6, 2]
UAV_team.setState(init_team)  # Set initial states

# This should be replaced by the designed control input sequence
u_traj = [[np.random.randn(4) for _ in range(num_uav)] for _ in range(T)]

# This simulates the UAV control and returns the final reward.
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

Total_reward = Total_reward - 100 * np.linalg.norm(numpy.array(UAV_team.getState()) - numpy.array(Target_state))  # Terminal cost.
print('Final Reward:')
print(Total_reward)
