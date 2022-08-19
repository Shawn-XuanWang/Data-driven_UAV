from casadi import *
from envS import Quadrotor

UAV1 = Quadrotor()
# UAV1.setRandomObstacles(n_obstacle=10, ball_r=1.0)
UAV1.setObstacles(
    [[0, 0, 7], [2, 4, 3], [4, 1, 9], [-3.5, 2, 7], [-2.5, -3.5, 5.5], [-2, 2, 10], [-3, -1, 4], [3, -4, 7],
     [-4, -5, 5]], 1.2)
Target_state = np.array([5, 5, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
# Do not change Target_state
dt = 0.05
# Do not change dt

T = 100
# Time horizon, can be set as arbitrary positive value.
# Depends on how long it takes the controller to drive the UAV to the target location [5, 5, 12]

init_x = np.array([-4.9, -5.1, 2.0,  # position r_I
          0, 0, 0,  # vel v_I
          0.6, 0.8, 0, 0,  # quat
          0, 0, 0,  # w_B
          ])
UAV1.setState(init_x)

# This should be replaced by the designed control input sequence
u_traj = np.random.randn(T, 4)

# This simulates the UAV control and returns the final reward. Do not change the following parts
Total_reward = 0
for ut in u_traj:
    UAV1.step(ut)
    current_risk = UAV1.RiskEval()
    if current_risk[0] > 0:  # obstacle violation.
        Total_reward = float('-inf')
    Total_reward = Total_reward - np.linalg.norm(ut) ** 2 - np.linalg.norm(
        UAV1.getState() - Target_state)  # Running cost.
    UAV1.render(dt * 3)  # 3 times slow motion

Total_reward = Total_reward - 100 * np.linalg.norm(UAV1.getState() - Target_state)  # Final cost.
print('Final Reward:')
print(Total_reward)
