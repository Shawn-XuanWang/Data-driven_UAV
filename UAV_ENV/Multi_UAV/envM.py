from casadi import *
import matplotlib.pyplot as plt
import math
import pickle


class Quadrotor:

    def __init__(self, ):

        # space size
        self.space_xyzmin = np.array([-10.0, -10.0, 0])
        self.space_xyzmax = np.array([10.0, 10.0, 15])
        self.target = np.array([5.0, 5.0, 12.0])
        # Target state: [5, 5 ,12 , 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]

        self.wing_len = 0.5

        with open('UAVa.pickle', 'rb') as handle:
            dyn_fn = pickle.load(handle)

        self.dyn_fn = dyn_fn

        # internal variable
        self.n_state = 13
        self.state = np.array([0, 0, 0,  # position r_I
                               0, 0, 0,  # vel v_I
                               1, 0, 0, 0,  # quat
                               0, 0, 0,  # w_B
                               ])
        self.control = np.array([0, 0, 0, 0])
        self.n_control = 4

        # obstacles
        self.obstacle_plot_info = None
        self.obstacle_info = None

        # other plot info
        self.line_arm1 = None
        self.line_arm2 = None
        self.line_arm3 = None
        self.line_arm4 = None

    def setState(self, state_val):
        assert len(state_val) == self.n_state, 'Wrong dimension'
        self.state = self._regularize_state(state_val)
        return self.state

    def getState(self):
        return self.state

    def step(self, control, dt=0.05):

        assert len(control) == self.n_control, 'Wrong control dimension!'
        assert dt <= 0.1, 'too large step size, should <=0.02 s'
        assert dt >= 0., 'too large step size, should >0'

        # do one-step Euler integration
        curr_state = self.getState()
        next_state = curr_state + dt * self.dyn_fn(curr_state, control).full().flatten()

        # normalize the quaternion in the next_state
        self.state = self._regularize_state(next_state)

        return self.state

    def render(self, time_pause=0.05):

        # create the plot object
        if not hasattr(self, 'ax'):
            self.fig = plt.figure(figsize=(8, 6))
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_xlabel('X (m)', fontsize=10, labelpad=5)
            self.ax.set_ylabel('Y (m)', fontsize=10, labelpad=5)
            self.ax.set_zlabel('Z (m)', fontsize=10, labelpad=5)
            self.ax.set_zlim(self.space_xyzmin[2], self.space_xyzmax[2])
            self.ax.set_ylim(self.space_xyzmin[1], self.space_xyzmax[1])
            self.ax.set_xlim(self.space_xyzmin[0], self.space_xyzmax[0])
            self.ax.set_box_aspect(aspect=self.space_xyzmax - self.space_xyzmin)

            # plot the obstacle (if any)
            if self.obstacle_plot_info is not None:
                for ob_surface in self.obstacle_plot_info:
                    self.ax.plot_surface(ob_surface[0], ob_surface[1], ob_surface[2])
            # plot the target
            self.ax.plot(self.target[0], self.target[1], self.target[2], linewidth=3, color='purple', marker='*',
                         markersize=10)

        else:
            self.line_arm1.remove()
            self.line_arm2.remove()
            self.line_arm3.remove()
            self.line_arm4.remove()

        # similar to hold on
        plt.ion()

        # get plot position
        plot_position = self._get_quadrotor_plot_position(self.getState())

        # plot
        c_x, c_y, c_z = plot_position[0:3]
        r1_x, r1_y, r1_z = plot_position[3:6]
        r2_x, r2_y, r2_z = plot_position[6:9]
        r3_x, r3_y, r3_z = plot_position[9:12]
        r4_x, r4_y, r4_z = plot_position[12:15]
        self.line_arm1, = self.ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=2, color='red', marker='o',
                                       markersize=3)
        self.line_arm2, = self.ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=2, color='blue', marker='o',
                                       markersize=3)
        self.line_arm3, = self.ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=2, color='red', marker='o',
                                       markersize=3)
        self.line_arm4, = self.ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=2, color='blue', marker='o',
                                       markersize=3)

        plt.pause(time_pause)

    def RiskEval(self):

        # if there is no obstacle
        if self.obstacle_info is None:
            return 0.0, {}

        # storage vector
        ob_dist = []
        is_safe = []
        vio_dist = []

        state = self.getState()
        uav_xyz = state[0:3]
        body_safe_margin = self.wing_len

        # compute the distance between uav and each obstacle
        for obstacle_i in self.obstacle_info:
            obstacle_xyz = obstacle_i[0]
            obstacle_r = obstacle_i[-1]
            ob_dis_i = np.linalg.norm(obstacle_xyz - uav_xyz)

            # store
            ob_dist.append(ob_dis_i)
            is_safe.append(ob_dis_i > obstacle_r + body_safe_margin)
            vio_dist.append(max(0.0, obstacle_r + body_safe_margin - ob_dis_i))

        # output
        detailed_info = dict(ob_dist=ob_dist,
                             is_safe=is_safe,
                             vio_dist=vio_dist)

        return (sum(vio_dist),
                detailed_info)

        # create the obstacles, we use 3d ball as obstacles

    def setRandomObstacles(self, n_obstacle=3, ball_r=1.0):

        self.obstacle_plot_info = []
        self.obstacle_info = []
        for i in range(n_obstacle):
            # ball position
            ball_xyz = np.random.uniform(low=np.array([-5.0, -5.0, 0]) + ball_r,
                                         high=np.array([5.0, 5.0, 15]) - ball_r,
                                         size=(3,))
            self.obstacle_info.append([ball_xyz, ball_r])

            # ball surface
            self.obstacle_plot_info.append(self._3DBall_surface(ball_xyz, ball_r))

    # allow you set the random obstacles
    def setObstacles(self, obstacle_xyz, ball_r):

        self.obstacle_plot_info = []
        self.obstacle_info = []

        n_obstacle = len(obstacle_xyz)
        for i in range(n_obstacle):
            ball_xyz = obstacle_xyz[i]
            self.obstacle_info.append([ball_xyz, ball_r])

            # ball surface
            self.obstacle_plot_info.append(self._3DBall_surface(ball_xyz, ball_r))

    # ---------------- internal utility function
    @staticmethod
    def _3DBall_surface(ball_xyz, ball_r):

        # Make data
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = ball_r * np.outer(np.cos(u), np.sin(v)) + ball_xyz[0]
        y = ball_r * np.outer(np.sin(u), np.sin(v)) + ball_xyz[1]
        z = ball_r * np.outer(np.ones(np.size(u)), np.cos(v)) + ball_xyz[2]

        return (x, y, z)

    def _get_quadrotor_plot_position(self, state):
        # thrust_position in body frame
        r1 = vertcat(self.wing_len / 2, 0, 0)
        r2 = vertcat(0, -self.wing_len / 2, 0)
        r3 = vertcat(-self.wing_len / 2, 0, 0)
        r4 = vertcat(0, self.wing_len / 2, 0)

        # position of COM
        rc = state[0:3]

        # altitude of quaternion
        q = state[6:10]
        q = q / (np.linalg.norm(q) + 0.00001)

        # direction cosine matrix from body to inertial
        CIB = np.transpose(self._dir_cosine(q).full())

        # position of each rotor in inertial frame
        r1_pos = rc + mtimes(CIB, r1).full().flatten()
        r2_pos = rc + mtimes(CIB, r2).full().flatten()
        r3_pos = rc + mtimes(CIB, r3).full().flatten()
        r4_pos = rc + mtimes(CIB, r4).full().flatten()

        # store
        plot_position = np.zeros(15)
        plot_position[0:3] = rc
        plot_position[3:6] = r1_pos
        plot_position[6:9] = r2_pos
        plot_position[9:12] = r3_pos
        plot_position[12:15] = r4_pos

        return plot_position

    def _regularize_state(self, state):
        # normalize the quaternion
        quat = state[6:10]
        state[6:10] = self._normalize_quat(quat)

        # clip the xyz
        xyz = state[0:3]
        state[0:3] = np.clip(xyz, a_min=self.space_xyzmin, a_max=self.space_xyzmax)

        return state

    @staticmethod
    def _normalize_quat(quat):
        return quat / np.linalg.norm(quat)

    @staticmethod
    def _dir_cosine(q):
        C_B_I = vertcat(
            horzcat(1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])),
            horzcat(2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])),
            horzcat(2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        )
        return C_B_I

    @staticmethod
    def _skew(v):
        v_cross = vertcat(
            horzcat(0, -v[2], v[1]),
            horzcat(v[2], 0, -v[0]),
            horzcat(-v[1], v[0], 0)
        )
        return v_cross

    @staticmethod
    def _omega(w):
        omeg = vertcat(
            horzcat(0, -w[0], -w[1], -w[2]),
            horzcat(w[0], 0, w[2], -w[1]),
            horzcat(w[1], -w[2], 0, w[0]),
            horzcat(w[2], w[1], -w[0], 0)
        )
        return omeg

    @staticmethod
    def _quaternion_mul(p, q):
        return vertcat(p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                       p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                       p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                       p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]
                       )


class Quadrotor_swarm(list):

    def __init__(self, num_uav):

        # space size
        self.num_uav = num_uav
        self.space_xyzmin = np.array([-10.0, -10.0, 0])
        self.space_xyzmax = np.array([10.0, 10.0, 15])
        self.target = [np.array([5.0, 5.0, 12.0]), np.array([6.0, 5.0, 12.0]), np.array([5.0, 6.0, 12.0]),
                       np.array([6.0, 6.0, 12.0])]
        # Target state: [5, 5 ,12 , 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]

        self.wing_len = 0.3
        self.n_state = 13
        self.n_control = 4

        # obstacles
        self.obstacle_plot_info = None
        self.obstacle_info = None

        self.dyn_fn = [None] * num_uav
        self.state = [None] * num_uav
        self.control = [None] * num_uav
        self.line_arms = [None] * num_uav
        with open('UAVa.pickle', 'rb') as handle:
            dyn_fn = pickle.load(handle)

        for i in range(self.num_uav):
            self.dyn_fn[i] = dyn_fn

            # internal variable
            self.state[i] = np.array([0, 0, 0,  # position r_I
                               0, 0, 0,  # vel v_I
                               1, 0, 0, 0,  # quat
                               0, 0, 0,  # w_B
                               ])
            self.control[i] = np.array([0, 0, 0, 0])

            self.line_arms[i] = [None] * 4


    def setState(self, state_val):
        print('state_val')
        print(state_val)
        assert len(state_val) == self.num_uav, 'Wrong number of UAVs'
        for i in range(self.num_uav):
            assert len(state_val[i]) == self.n_state, 'Wrong state dimension!'
            self.state[i] = self._regularize_state(state_val[i])
        return self.state

    def getState(self):
        return self.state

    def step(self, control, dt=0.05):
        assert len(control) == self.num_uav, 'Wrong number of UAVs'

        curr_state = self.getState()
        for i in range(self.num_uav):
            assert len(control[0]) == self.n_control, 'Wrong control dimension!'

            # do one-step Euler integration
            next_state = curr_state[i] + dt * self.dyn_fn[i](curr_state[i], control[i]).full().flatten()

            # normalize the quaternion in the next_state
            self.state[i] = self._regularize_state(next_state)

        return self.state

    def render(self, time_pause=0.05):

        # create the plot object
        if not hasattr(self, 'ax'):
            self.fig = plt.figure(figsize=(8, 6))
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_xlabel('X (m)', fontsize=10, labelpad=5)
            self.ax.set_ylabel('Y (m)', fontsize=10, labelpad=5)
            self.ax.set_zlabel('Z (m)', fontsize=10, labelpad=5)
            self.ax.set_zlim(self.space_xyzmin[2], self.space_xyzmax[2])
            self.ax.set_ylim(self.space_xyzmin[1], self.space_xyzmax[1])
            self.ax.set_xlim(self.space_xyzmin[0], self.space_xyzmax[0])
            self.ax.set_box_aspect(aspect=self.space_xyzmax - self.space_xyzmin)

            # plot the obstacle (if any)
            if self.obstacle_plot_info is not None:
                for ob_surface in self.obstacle_plot_info:
                    self.ax.plot_surface(ob_surface[0], ob_surface[1], ob_surface[2])
            # plot the targets
            for i in range(self.num_uav):
                self.ax.plot(self.target[i][0], self.target[i][1], self.target[i][2], linewidth=3, color='purple', marker='*',
                         markersize=10)

        else:
            for i in range(self.num_uav):
                for k in range(4):
                    self.line_arms[i][k].remove()


        # similar to hold on
        plt.ion()

        for i in range(self.num_uav):
            # get plot position
            plot_position = self._get_quadrotor_plot_position(self.getState()[i])

            # plot
            c_x, c_y, c_z = plot_position[0:3]
            r1_x, r1_y, r1_z = plot_position[3:6]
            r2_x, r2_y, r2_z = plot_position[6:9]
            r3_x, r3_y, r3_z = plot_position[9:12]
            r4_x, r4_y, r4_z = plot_position[12:15]
            self.line_arms[i][0], = self.ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=2, color='red', marker='o',
                                           markersize=3)
            self.line_arms[i][1], = self.ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=2, color='blue', marker='o',
                                           markersize=3)
            self.line_arms[i][2], = self.ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=2, color='red', marker='o',
                                           markersize=3)
            self.line_arms[i][3], = self.ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=2, color='blue', marker='o',
                                           markersize=3)

        plt.pause(time_pause)

    def RiskEval(self):

        # if there is no obstacle
        if self.obstacle_info is None:
            return 0.0, {}

        # storage vector
        ob_dist = [[] for _ in range(self.num_uav)]
        ob_is_safe = [[] for _ in range(self.num_uav)]
        ob_vio_dist = [[] for _ in range(self.num_uav)]
        uav_dist = [[] for _ in range(self.num_uav)]
        uav_is_safe = [[] for _ in range(self.num_uav)]
        uav_vio_dist = [[] for _ in range(self.num_uav)]

        state = self.getState()
        for i in range(self.num_uav):
            uav_xyz = state[i][0:3]
            body_safe_margin = self.wing_len

            # compute the distance between uav and each obstacle
            for obstacle_i in self.obstacle_info:
                obstacle_xyz = obstacle_i[0]
                obstacle_r = obstacle_i[-1]
                ob_dis_i = np.linalg.norm(obstacle_xyz - uav_xyz)

                # store
                ob_dist[i].append(ob_dis_i)
                ob_is_safe[i].append(ob_dis_i > obstacle_r + body_safe_margin)
                ob_vio_dist[i].append(max(0.0, obstacle_r + body_safe_margin - ob_dis_i))

            # compute the distance among uavs
            for uav_j in range(self.num_uav):
                if uav_j != i:
                    uav_j_xyz = state[uav_j][0:3]
                    uav_dis_ij = np.linalg.norm(uav_j_xyz - uav_xyz)

                    # store
                    uav_dist[i].append(uav_dis_ij)
                    uav_is_safe[i].append(uav_dis_ij > 2 * body_safe_margin)
                    uav_vio_dist[i].append(max(0.0, 2 * body_safe_margin - uav_dis_ij))


        # output
        detailed_info = dict(ob_dist=ob_dist,
                            ob_is_safe=ob_is_safe,
                            ob_vio_dist=ob_vio_dist,
                             uav_dist=uav_dist,
                             uav_is_safe=uav_is_safe,
                             uav_vio_dist=uav_vio_dist,
                             )

        return ([sum(ob_vio_dist[i]) for i in range(self.num_uav)],
                [sum(uav_vio_dist[i]) for i in range(self.num_uav)],
                detailed_info)

        # create the obstacles, we use 3d ball as obstacles

    def setRandomObstacles(self, n_obstacle=3, ball_r=1.0):

        self.obstacle_plot_info = []
        self.obstacle_info = []
        for i in range(n_obstacle):
            # ball position
            ball_xyz = np.random.uniform(low=np.array([-5.0, -5.0, 0]) + ball_r,
                                         high=np.array([5.0, 5.0, 15]) - ball_r,
                                         size=(3,))
            self.obstacle_info.append([ball_xyz, ball_r])

            # ball surface
            self.obstacle_plot_info.append(self._3DBall_surface(ball_xyz, ball_r))

    # allow you set the random obstacles
    def setObstacles(self, obstacle_xyz, ball_r):

        self.obstacle_plot_info = []
        self.obstacle_info = []

        n_obstacle = len(obstacle_xyz)
        for i in range(n_obstacle):
            ball_xyz = obstacle_xyz[i]
            self.obstacle_info.append([ball_xyz, ball_r])

            # ball surface
            self.obstacle_plot_info.append(self._3DBall_surface(ball_xyz, ball_r))

    # ---------------- internal utility function
    @staticmethod
    def _3DBall_surface(ball_xyz, ball_r):

        # Make data
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = ball_r * np.outer(np.cos(u), np.sin(v)) + ball_xyz[0]
        y = ball_r * np.outer(np.sin(u), np.sin(v)) + ball_xyz[1]
        z = ball_r * np.outer(np.ones(np.size(u)), np.cos(v)) + ball_xyz[2]

        return (x, y, z)

    def _get_quadrotor_plot_position(self, state):
        # thrust_position in body frame
        r1 = vertcat(self.wing_len / 2, 0, 0)
        r2 = vertcat(0, -self.wing_len / 2, 0)
        r3 = vertcat(-self.wing_len / 2, 0, 0)
        r4 = vertcat(0, self.wing_len / 2, 0)

        # position of COM
        rc = state[0:3]

        # altitude of quaternion
        q = state[6:10]
        q = q / (np.linalg.norm(q) + 0.00001)

        # direction cosine matrix from body to inertial
        CIB = np.transpose(self._dir_cosine(q).full())

        # position of each rotor in inertial frame
        r1_pos = rc + mtimes(CIB, r1).full().flatten()
        r2_pos = rc + mtimes(CIB, r2).full().flatten()
        r3_pos = rc + mtimes(CIB, r3).full().flatten()
        r4_pos = rc + mtimes(CIB, r4).full().flatten()

        # store
        plot_position = np.zeros(15)
        plot_position[0:3] = rc
        plot_position[3:6] = r1_pos
        plot_position[6:9] = r2_pos
        plot_position[9:12] = r3_pos
        plot_position[12:15] = r4_pos

        return plot_position

    def _regularize_state(self, state):
        # normalize the quaternion
        quat = state[6:10]
        state[6:10] = self._normalize_quat(quat)

        # clip the xyz
        xyz = state[0:3]
        state[0:3] = np.clip(xyz, a_min=self.space_xyzmin, a_max=self.space_xyzmax)

        return state

    @staticmethod
    def _normalize_quat(quat):
        return quat / np.linalg.norm(quat)

    @staticmethod
    def _dir_cosine(q):
        C_B_I = vertcat(
            horzcat(1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])),
            horzcat(2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])),
            horzcat(2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        )
        return C_B_I

    @staticmethod
    def _skew(v):
        v_cross = vertcat(
            horzcat(0, -v[2], v[1]),
            horzcat(v[2], 0, -v[0]),
            horzcat(-v[1], v[0], 0)
        )
        return v_cross

    @staticmethod
    def _omega(w):
        omeg = vertcat(
            horzcat(0, -w[0], -w[1], -w[2]),
            horzcat(w[0], 0, w[2], -w[1]),
            horzcat(w[1], -w[2], 0, w[0]),
            horzcat(w[2], w[1], -w[0], 0)
        )
        return omeg

    @staticmethod
    def _quaternion_mul(p, q):
        return vertcat(p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                       p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                       p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                       p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]
                       )


# converter to quaternion from (angle, direction)
def angleAxis2Quaternion(angle, dir):
    if type(dir) == list:
        dir = numpy.array(dir)
    dir = dir / numpy.linalg.norm(dir)
    quat = numpy.zeros(4)
    quat[0] = math.cos(angle / 2)
    quat[1:] = math.sin(angle / 2) * dir
    return quat


# normalized verctor
def normalizeVec(vec):
    if type(vec) == list:
        vec = np.array(vec)
    vec = vec / np.linalg.norm(vec)
    return vec


def quaternion_conj(q):
    conj_q = q
    conj_q[1] = -q[1]
    conj_q[2] = -q[2]
    conj_q[3] = -q[3]
    return conj_q
