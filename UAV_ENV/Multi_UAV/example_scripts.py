from casadi import *
from envM import Quadrotor_swarm

# Initial the environment
num_uav = 4    # number of UAVs is fixed, do not change
UAV_team = Quadrotor_swarm(num_uav)
UAV_team.setRandomObstacles(n_obstacle=2, ball_r=1.4)

# Set initial states
init_team = [np.random.randn(UAV_team.n_state) for _ in range(num_uav)] # UAV_team.n_state = 13
init_team[0][0:3] = np.array([-5, -5, 2] + 0.5*np.random.randn(3))  # Set position of UAV0 close to [-5, -5, 2]
init_team[1][0:3] = np.array([-6, -5, 2] + 0.5*np.random.randn(3))  # Set position of UAV1 close to [-6, -5, 2]
init_team[2][0:3] = np.array([-5, -6, 2] + 0.5*np.random.randn(3))  # Set position of UAV2 close to [-5, -6, 2]
init_team[3][0:3] = np.array([-6, -6, 2] + 0.5*np.random.randn(3))  # Set position of UAV3 close to [-6, -6, 2]
UAV_team.setState(init_team)  # Set initial states

# =============== The followings are example scripts to call and receive feedback from the environment. ==============


# Set(state_val)
UAV_team.setState([np.random.randn(UAV_team.n_state) for _ in range(num_uav)])


# Get()
current_state = UAV_team.getState()
print('current_state: ' + str(current_state))


# Step(input)
next_state = UAV_team.step([np.random.randn(4) for _ in range(num_uav)])
print('next_state: ' + str(next_state))


# Reset()
UAV_team.setState(init_team)


# RiskEval()
current_risk = UAV_team.RiskEval()
print('current_risk: ' + str(current_risk))
# ( sum of obstacle violation  {
# ob_dist: distances to each obstacle,
# is_safe: the violation with each obstacle,
# vio_dist: the distance that the UAV runs into each obstacle)})


# Render()
UAV_team.render(time_pause=100)
# after time_pause (unit in sec), the rendered environment closes automatically, use it for slow motion video
