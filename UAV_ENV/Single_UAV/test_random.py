from casadi import *
from envS import Quadrotor

UAV1 = Quadrotor()
UAV1.setRandomObstacles(n_obstacle=10, ball_r=1.2)
Target_state = np.array([5, 5 ,12 , 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
dt = 0.05

T = 100
# Time horizon, can be set as arbitrary positive value.
# Depends on how long it takes the controller to drive the UAV to the target location [5, 5, 12]
init_x = np.random.randn(UAV1.n_state)
init_x[0:3] = np.array([-5, -5, 2] + np.random.randn(3))
UAV1.setState(init_x)



# This should be replaced by the designed control input sequence
u_traj = np.random.randn(T, 4)

# This simulates the UAV control and returns the final reward.
Total_reward = 0
for ut in u_traj:
    UAV1.step(ut)
    current_risk = UAV1.RiskEval()
    if current_risk[0] > 0:     # obstacle violation.
        Total_reward = float('-inf')
    Total_reward = Total_reward - np.linalg.norm(ut)**2 - np.linalg.norm(UAV1.getState()-Target_state)   # Running cost.
    UAV1.render(dt * 3)   # 3 times slow motion

Total_reward = Total_reward - 100 * np.linalg.norm(UAV1.getState()-Target_state)    # Terminal cost.
print('Final Reward:')
print(Total_reward)