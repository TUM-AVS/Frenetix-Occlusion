__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
# own
from frenetix_occlusion.route_planner import FORoutePlanner
from frenetix_occlusion.utils.frenetix_handler import FrenetixHandler
import frenetix_occlusion.utils.helper_functions as hf

# commonroad
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import InitialState, CustomState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory

# general
import numpy as np
from random import randint
import warnings


class FOAgentManager:
    def __init__(self, scenario, reference_path, config, timestep, visualization=None, dt=0.1, fo_obstacles=None, debug=False):
        self.scenario = scenario
        self.timestep = timestep
        self.reference_path = reference_path
        self.config = config
        self.visualization = visualization
        self.fo_obstacles = fo_obstacles
        self.dt = dt
        self.debug = debug
        self.phantom_agents = []
        self.real_agents = []
        self.predictions = {}

        # get all obstacle ids
        self._get_all_ids()

    def reset(self):
        self.phantom_agents = []
        self.predictions = {}

    def add_agent(self, pos, velocity='default', agent_type='Car', add_to_scenario=False,
                  timestep=0, horizon=3.0, mode='ref_path', orientation=None):
        """
            This method creates and configures an agent of a given type (Car, Bicycle, Pedestrian) with
            specific parameters such as position, velocity, and physical characteristics. The agent is
            either added to the real scenario or maintained as a phantom agent based on the 'add_to_scenario' flag.

            Parameters:
            - pos (ndarray): The initial position (x, y) of the agent in the simulation environment.
            - velocity (str, int, float, optional): The velocity of the agent. This can be:
                - 'default': Sets a predefined default velocity based on the agent type.
                - 'lanelet': Specific to 'Car' type, computes velocity based on lanelet data
                - numeric value (int/float): Sets a specific velocity value.
            - agent_type (str, optional): The type of agent to create. Valid options are 'Car', 'Bicycle', or 'Pedestrian'.
            - add_to_scenario (bool, optional): If True, the agent is added to the real scenario.
                                                If False, it is kept as a phantom agent for simulation purposes.

            Returns:
            - An instance of the created agent (OAPVehicleAgent or OAPPedestrianAgent).
            """

        if self.timestep != timestep:
            return

        agent_id = self._create_id()
        # define agent size based on agent type
        if agent_type.lower() == "bicycle":
            conf = self.config['bicycle']
            agent_params = {'agent_id': agent_id, 'type': 'Bicycle', 'width': conf['width'], 'length': conf['length'],
                            'delta_max': 1.066, 'wheelbase': conf['wheelbase'], 'switching_velocity': 4, 'max_acceleration': 11.5,
                            'velocity_delta_max': 0.4}

            # define the desired agent velocity
            if velocity == 'default':
                velocity = conf['default_velocity']
            elif velocity == 'lanelet':
                raise NotImplementedError
            elif not isinstance(velocity, float) and not isinstance(velocity, int):
                raise ValueError('Only "default", int or float is allowed!')

        elif agent_type.lower() == "car":
            conf = self.config['car']
            agent_params = {'agent_id': agent_id, 'type': 'Car', 'width': conf['width'], 'length': conf['length'],
                            'delta_max': 1.066, 'wheelbase': conf['wheelbase'], 'switching_velocity': 7.32,
                            'max_acceleration': 11.5, 'velocity_delta_max': 0.4}

            # define the desired agent velocity
            if velocity == 'default':
                velocity = conf['default_velocity']
            elif velocity == 'lanelet':
                velocity = OAPVehicleAgent.get_lanelet_velocity(pos=pos, scenario=self.scenario)
            elif not isinstance(velocity, float) and not isinstance(velocity, int):
                raise ValueError('Only "default", "lanelet", int or float is allowed!')

        elif agent_type.lower() == "truck":
            conf = self.config['truck']
            agent_params = {'agent_id': agent_id, 'type': 'Truck', 'width': conf['width'], 'length': conf['length'],
                            'delta_max': 1.066, 'wheelbase': conf['wheelbase'], 'switching_velocity': 7.32, 'max_acceleration': 7,
                            'velocity_delta_max': 0.4}

            # define the desired agent velocity
            if velocity == 'default':
                velocity = conf['default_velocity']
            elif velocity == 'lanelet':
                velocity = OAPVehicleAgent.get_lanelet_velocity(pos=pos, scenario=self.scenario)
            elif not isinstance(velocity, float) and not isinstance(velocity, int):
                raise ValueError('Only "default", "lanelet", int or float is allowed!')

        elif agent_type.lower() == "pedestrian":
            conf = self.config['pedestrian']
            agent_params = {'agent_id': agent_id, 'type': 'Pedestrian', 'width': conf['width'], 'length': conf['length']}
        else:
            raise NotImplementedError(f'OAPManager: Agent type "{agent_type}" is not implemented!')

        # create agent and add to list
        if agent_type == "Pedestrian":

            # define the desired agent velocity
            if velocity == 'default':
                velocity = conf['default_velocity']
            elif velocity == 'lanelet':
                raise NotImplementedError
            elif not isinstance(velocity, float) and not isinstance(velocity, int):
                raise ValueError('Only "default", int or float is allowed!')
            agent = OAPPedestrianAgent(pos=pos, velocity=velocity, agent_type=agent_type, agent_params=agent_params,
                                       scenario=self.scenario, dt=self.dt, horizon=horizon, ref_path=self.reference_path,
                                       visualization=self.visualization, debug=self.debug, mode=mode, orientation=orientation,
                                       config=self.config)

        else:
            agent = OAPVehicleAgent(pos=pos, velocity=velocity, agent_type=agent_type, agent_params=agent_params,
                                    scenario=self.scenario, dt=self.dt, horizon=horizon, visualization=self.visualization,
                                    debug=self.debug, config=self.config)

        if add_to_scenario:
            # add agent to real agents list
            self.real_agents.append(agent)

            # add agent to commonroad scenario
            agent.add_to_commonroad_scenario(timestep=timestep)

            # additionally, add the agent to fo obstacles for visible area calculation
            if self.fo_obstacles is not None:
                self.fo_obstacles.add(agent.commonroad_dynamic_obstacle)
        else:
            self.phantom_agents.append(agent)
            # update phantom agent predictions
            self._add_prediction(agent)

        return agent

    def agent_by_prediction_id(self, prediction_id):
        if not self.phantom_agents:
            return

        # only use first five digits of the id
        agent_id = int(str(prediction_id)[:5])

        # iterate over agents and return the desired agent
        for agent in self.phantom_agents:
            if agent.agent_id == agent_id:
                return agent

    def update_real_agents(self, cr_scenario_predictions):
        # update prediction dict of pedestrians and cyclists (walenet cannot predict them properly)
        for agent in self.real_agents:
            if agent.agent_type.lower() == 'pedestrian':
                if agent.agent_id in cr_scenario_predictions:
                    agent.predictions = agent._create_cr_predictions(self.timestep)
                    cr_scenario_predictions[agent.agent_id] = agent.predictions[0]

    def _add_prediction(self, agent):
        # adds the prediction of an agent to predictions
        for i, prediction in enumerate(agent.predictions):
            prediction_id = int(str(agent.agent_id) + str(i))
            self.predictions[prediction_id] = prediction

    def _get_all_ids(self):
        self.all_obstacle_id = [obst.obstacle_id for obst in self.scenario.obstacles]

    def _create_id(self):
        # create random number
        agent_id = randint(10000, 11000)

        # check if number already exists in all ids
        if agent_id in self.all_obstacle_id:
            agent_id = self._create_id()

        # add id to all ids
        self.all_obstacle_id.append(agent_id)

        return agent_id


class OAPAgent:
    def __init__(self, pos, velocity, agent_type, agent_params, scenario, config,
                 dt=0.1, horizon=3.0, visualization=None, debug=False):
        self.cr_scenario = scenario
        self.dt = dt
        self.horizon = horizon
        self.agent_id = agent_params['agent_id']
        self.visualization = visualization
        self.debug = debug
        self.initial_position = pos
        self.initial_velocity = velocity
        self.agent_type = agent_type
        self.obstacle_type = ObstacleType(self.agent_type.lower())
        self.agent_params = agent_params
        self.shape = Rectangle(agent_params["length"], agent_params["width"], center=np.array([0.0, 0.0]),
                               orientation=0.0)
        self.initial_state = None
        self.predictions = None
        self.config = config

        # commonroad object (if agent shall be added to scenario)
        self.commonroad_dynamic_obstacle = None

    def add_to_commonroad_scenario(self, timestep=0):

        # load initial state
        initial_state = self.initial_state

        # set current time step
        initial_state.time_step = timestep

        # create commonroad trajectory prediction
        cr_state_list = self._create_cr_state_list(timestep)

        # create commonroad trajectory from state list
        cr_trajectory = Trajectory(initial_time_step=timestep+1, state_list=cr_state_list)

        # create trajectory prediction from trajectory
        cr_trajectory_prediction = TrajectoryPrediction(trajectory=cr_trajectory, shape=self.shape)

        # combine all information to commonroad dynamic obstacle
        self.commonroad_dynamic_obstacle = DynamicObstacle(obstacle_id=self.agent_id,
                                                           obstacle_type=self.obstacle_type,
                                                           obstacle_shape=self.shape,
                                                           initial_state=initial_state,
                                                           prediction=cr_trajectory_prediction)

        # add dynamic obstacle to commonroad scenario
        self.cr_scenario.add_objects(self.commonroad_dynamic_obstacle)

        return

    def _create_cr_state_list(self, time_step):
        pass

    def _create_cr_predictions(self, timestep):
        pass

    @staticmethod
    def create_cov_matrix(pos_list, initial_variance=1.0, variance_factor=1.1):
        """
        Creates a list of covariance matrices for each position in the trajectory,
        with an exponential increase in variance.

        :param pos_list: List of positions (x, y) of the trajectory.
        :param initial_variance: Initial variance value.
        :param variance_factor: Factor by which variance increases exponentially.
        :return: Array of covariance matrices.
        """
        # Number of positions in the trajectory
        num_positions = len(pos_list)

        # Calculate exponential variances for all positions
        variances = initial_variance * np.power(variance_factor, np.arange(num_positions))

        # Create covariance matrices with the same variance in X and Y
        cov_matrix = np.array([[[var, 0], [0, var]] for var in variances])

        return cov_matrix


class OAPVehicleAgent(OAPAgent):
    def __init__(self, pos, velocity, agent_type, agent_params, scenario, config,
                 dt=0.1, horizon=3.0, visualization=None, debug=False):

        # call constructor of super class
        super().__init__(pos, velocity, agent_type, agent_params, scenario, config, dt, horizon, visualization=visualization,
                         debug=debug)

        # own attributes
        self.route_planner = FORoutePlanner(scenario, scenario.lanelet_network, visualization, debug)

        # create reference paths using the route planner
        self.reference_paths = self.route_planner.calc_possible_reference_paths(pos)

        # create a commonroad-initial-state using the position and the orientation from the lanelet
        self.initial_state = InitialState(position=self.initial_position,
                                          orientation=self.route_planner.lanelet_orientation,
                                          velocity=self.initial_velocity)

        # create the frenetix handlers and the coordinates
        self.frenetix_handlers = [handler for rp in self.reference_paths
                                  if (handler := FrenetixHandler(dt=self.dt, ref_path=rp, agent_params=self.agent_params,
                                                                 initial_state=self.initial_state, horizon=horizon)).is_valid]

        # create trajectories
        for handler in self.frenetix_handlers:
            handler.create_trajectories(horizon=horizon)

        # create prediction
        self.predictions = self._create_cr_predictions(0)

    def draw_debug_plot(self, color='k'):
        if self.visualization is None:
            return

        self.visualization.draw_point(self.initial_position, color=color, zorder=20)
        for ref_path in self.reference_paths:
            self.visualization.draw_reference_path(ref_path, zorder=20)

        for handler in self.frenetix_handlers:
            self.visualization.plot_trajectories(handler.trajectories, zorder=20, color=color)

    @staticmethod
    def get_lanelet_velocity(pos, scenario):
        # find current lanelet id and lanelet
        lanelet_id = scenario.lanelet_network.find_lanelet_by_position([pos])[0][0]
        lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)

        # extract lanelet type
        lanelet_type = list(lanelet.lanelet_type)[0].value

        if lanelet_type == 'urban':
            # 30 km/h in urban environment
            v = 30/3.6
        elif lanelet_type == 'country':
            # 80 km/h on country roads (take me home, to the place, I belong ....)
            v = 80/3.6
        elif lanelet_type == 'highway':
            # 100 km/h on highways
            v = 100/3.6
        else:
            v = 50 / 3.6
            warnings.warn(f'OAPAgent: Lanelet type "{lanelet_type}" is not implemented yet! Default value of 50 km/h will be used!')

        return v

    def _select_trajectory(self, handler=None, mode='constant_velocity'):

        # if no handler is available, the handler that goes straight is selected
        if handler is None:
            min_variance = float('inf')  # Initialize with infinity for comparison
            selected_handler = None
            for handler in self.frenetix_handlers:
                variance = np.var(handler.coordinate_system_cpp.ref_theta)

                # Check if this variance is smaller than the smallest variance found so far
                if variance < min_variance:
                    min_variance = variance
                    selected_handler = handler
            handler = selected_handler

        if mode == 'constant_velocity':
            min_variance = float('inf')  # Initialize with infinity for comparison
            selected_trajectory = None

            for trajectory in handler.trajectories:
                # Calculate the variance of the velocities for the current trajectory
                variance = np.var(trajectory.cartesian.v)

                # Check if this variance is smaller than the smallest variance found so far
                if variance < min_variance:
                    min_variance = variance
                    selected_trajectory = trajectory
        else:
            raise NotImplementedError

        return selected_trajectory

    def _create_cr_state_list(self, time_step):

        # select desired trajectory
        trajectory = self._select_trajectory()

        # initialize state list and append custom states
        state_list = []
        for i in range(1, len(trajectory.cartesian.x)):
            position = np.array([trajectory.cartesian.x[i], trajectory.cartesian.y[i]])
            custom_state = CustomState(orientation=trajectory.cartesian.theta[i],
                                       velocity=trajectory.cartesian.v[i],
                                       position=position,
                                       time_step=time_step + i)

            state_list.append(custom_state)
        return state_list

    def _create_cr_predictions(self, timestep) -> list:
        # create dict like a commonroad prediction (e.g. walenet) -> needed in harm estimation
        predictions = []
        # create shape with buffer
        if self.agent_type == "Bicycle":
            shape = {'length': self.shape.length * self.config['prediction']['size_factor_length_l'],
                     'width': self.shape.width * self.config['prediction']['size_factor_width_l']}
        else:
            shape = {'length': self.shape.length * self.config['prediction']['size_factor_length_s'],
                     'width': self.shape.width * self.config['prediction']['size_factor_width_s']}

        # iterate over all trajectories in all handlers (different ref paths)
        for handler in self.frenetix_handlers:
            traj = self._select_trajectory(handler, mode='constant_velocity')

            pos_list = np.column_stack((traj.cartesian.x[timestep:], traj.cartesian.y[timestep:]))
            v_list = traj.cartesian.v[timestep:]
            orientation_list = traj.cartesian.theta[timestep:]
            cov_list = self.create_cov_matrix(pos_list, initial_variance=0.1,
                                              variance_factor=self.config['prediction']['variance_factor'])

            # combine lists to dict and append to list
            predictions.append({'orientation_list': orientation_list,
                                'v_list': v_list,
                                'pos_list': pos_list,
                                'shape': shape,
                                'cov_list': cov_list})

        return predictions


class OAPPedestrianAgent(OAPAgent):
    def __init__(self, pos, velocity, agent_type, agent_params, scenario, config, dt=0.1, horizon=3.0, ref_path=None,
                 visualization=None, debug=False, mode='ref_path', orientation=None):
        # call constructor of super class
        super().__init__(pos, velocity, agent_type, agent_params, scenario, config, dt, horizon, visualization, debug)
        self.ego_reference_path = ref_path

        self.reference_curve = None
        self.trajectory = self._create_ped_trajectory(mode=mode, horizon=horizon, orientation=orientation)

        # create prediction
        self.predictions = self._create_cr_predictions(0)

    def draw_debug_plot(self, color='k'):
        if self.visualization is None:
            return

        # draw initial, position reference line, end point and line
        self.visualization.rnd.ax.plot(self.initial_position[0], self.initial_position[1], color=color, marker='o', zorder=100)
        self.visualization.rnd.ax.plot(self.reference_curve[:, 0], self.reference_curve[:, 1], color='y', zorder=100)
        self.visualization.rnd.ax.plot(self.trajectory[:, 0], self.trajectory[:, 1], color=color, marker='o', zorder=100)

    def _create_ped_trajectory(self, mode='ref_path', horizon=3.0, orientation=None):

        if mode == 'lane_center':
            # find current lanelet id and lanelet
            start_lanelet_id = self.cr_scenario.lanelet_network.find_lanelet_by_position([self.initial_position])

            if not len(start_lanelet_id[0]) == 0:
                # select lanelet and extract lanelet_center
                start_lanelet = self.cr_scenario.lanelet_network.find_lanelet_by_id(start_lanelet_id[0][0])
                curve = np.array(start_lanelet.center_vertices)

            # if point is not within the lanelet, use mode 'ref_path'
            else:
                self._create_ped_trajectory(mode='ref_path')
                return

        elif mode == 'ref_path':
            # select reference path as curve
            curve = self.ego_reference_path

        else:
            raise NotImplementedError(f'Selected mode "{mode}" is not implemented: use "ref_path" or "lane_center"!')

        # check if orientation is provided by spawn point, otherwise calculate own orientation
        if orientation is None:
            # calc normal vector towards curve and store it in object
            normal_vector = hf.calc_normal_vector_to_curve(curve, self.initial_position)
            self.reference_curve = curve

            # calculate orientation towards lane center / reference path
            initial_orientation = hf.angle_between_positive(np.array([1, 0]), normal_vector)

        else:
            initial_orientation = orientation

        # create a commonroad-initial-state using the position and the orientation
        self.initial_state = InitialState(position=self.initial_position,
                                          orientation=initial_orientation,
                                          velocity=self.initial_velocity)

        # Compute the x and y components of the velocity
        vx = round(self.initial_state.velocity * np.cos(self.initial_state.orientation), 3)
        vy = round(self.initial_state.velocity * np.sin(self.initial_state.orientation), 3)

        # Compute the number of time steps
        num_steps = int(horizon / self.dt) + 1

        # Create a matrix of time steps and velocities
        t = np.arange(num_steps)[:, np.newaxis] * self.dt
        v_matrix = np.array([vx, vy])

        # Compute the trajectory
        trajectory = self.initial_position + t * v_matrix.T

        return trajectory

    def _create_cr_state_list(self, time_step):

        # initialize state list and append custom states
        state_list = []
        for i in range(1, len(self.trajectory)):
            custom_state = CustomState(orientation=self.initial_state.orientation,
                                       velocity=self.initial_velocity,
                                       position=self.trajectory[i],
                                       time_step=time_step + i)

            state_list.append(custom_state)
        return state_list

    def _create_cr_predictions(self, timestep) -> list:
        # create dict like a commonroad prediction (e.g. walenet) -> needed in harm estimation
        pos_list = self.trajectory[timestep:]
        v_list = np.full(len(pos_list), self.initial_state.velocity)
        orientation_list = np.full(len(pos_list), self.initial_state.orientation)
        shape = {'length': self.shape.length * self.config['prediction']['size_factor_length_s'],
                 'width': self.shape.width * self.config['prediction']['size_factor_width_s']}
        cov_list = self.create_cov_matrix(pos_list, initial_variance=0.1,
                                          variance_factor=self.config['prediction']['variance_factor'])

        predictions = {'orientation_list': orientation_list,
                       'v_list': v_list,
                       'pos_list': pos_list,
                       'shape': shape,
                       'cov_list': cov_list}

        return [predictions]
# eof
