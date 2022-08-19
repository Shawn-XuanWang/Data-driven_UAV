from casadi import *
from envS import Quadrotor

# Initial the environment
UAV1 = Quadrotor()
UAV1.setRandomObstacles(n_obstacle=1, ball_r=1.2)

# Set initial states
init_x = np.random.randn(UAV1.n_state)  # env.n_state = 13
init_x[0:3] = np.array([-5, -5, 2] + np.random.randn(3))  # Set position of UAV close to [-5, -5, 2]
UAV1.setState(init_x)  # Set initial states

# =============== The followings are example scripts to call and receive feedback from the environment. ==============


# Set(state_val)
UAV1.setState(np.random.randn(UAV1.n_state))


# Get()
current_state = UAV1.getState()
print('current_state: ' + str(current_state))


# Step(input)
next_state = UAV1.step(np.random.randn(4))
print('next_state: ' + str(next_state))


# Reset()
UAV1.setState(init_x)


# RiskEval()
current_risk = UAV1.RiskEval()
print('current_risk: ' + str(current_risk))
# ( sum of obstacle violation  {
# ob_dist: distances to each obstacle,
# is_safe: the violation with each obstacle,
# vio_dist: the distance that the UAV runs into each obstacle)})


# Render()
UAV1.render(time_pause=100)
# after time_pause (unit in sec), the rendered environment closes automatically, use it for slow motion video
