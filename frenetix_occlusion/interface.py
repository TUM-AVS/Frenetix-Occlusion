__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
from frenetix_occlusion.sensor_model import SensorModel
from frenetix_occlusion.agent import FOAgentManager
from frenetix_occlusion.spawn_locator import SpawnLocator
from frenetix_occlusion.metrics.metric import Metric
from frenetix_occlusion.utils.fo_obstacle import FOObstacles
from frenetix_occlusion.utils.visualization import FOVisualization
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
import yaml, os


class FOInterface:
    """
        The FOInterface class is the main interface of Frenetix-Occlusion.

        Attributes:
            - config (dict): Configuration settings loaded from a config file
            - cr_scenario (Scenario): The current scenario being evaluated, including the lanelet network and obstacles.
            - lanelet_network (LaneletNetwork): The lanelet network of the current scenario.
            - ego_reference_path (list): The reference path for the ego vehicle.
            - cosy_cl (CurvilinearCoordinateSystem): The curvilinear coordinate system based on the ego vehicle's reference path.
              Defaults to None and is initialized if not provided.
            - vehicle_params (dict): Parameters of the ego vehicle, including dimensions and dynamics.
            - dt (float): The simulation time step in seconds.
            - plot (bool): Flag to enable or disable plotting for debug visualization.
            - predictions (dict): Predictions about other agents in the environment, updated during scenario evaluation.
            - ego_pos, ego_orientation, ego_pos_cl (tuple, float, tuple): The ego vehicle's position, orientation, and position
            - timestep (int): The current timestep of the simulation
            - spawn_points (list): Potential spawn points for phantom agents, identified during scenario evaluation.
            - sensor_radius, sensor_angle (float): Parameters of the sensor model defining the sensing range and angle.
            - visualization (FOVisualization): Tool for visualizing the scenario and for debugging purposes
            - fo_obstacles (FOObstacles): A representation of obstacles detected by the sensor model.
            - sensor_model (SensorModel): The sensor model used to detect obstacles and calculate visible and occluded areas.
            - agent_manager (FOAgentManager): Manages agents (both real and phantom) within the simulation.
            - spawn_locator (SpawnLocator): Identifies potential spawn points for phantom agents based on the current
              scenario state.
            - metrics (Metric): Metrics for evaluating the performance and safety of the ego vehicle in the scenario.

        Initialization Parameters:
            - scenario (Scenario): The current scenario, including lanelets and obstacles.
            - reference_path (list): The reference path for the ego vehicle.
            - vehicle_params (dict): Parameters of the ego vehicle.
            - dt (float): The simulation time step in seconds.
            - cosy_cl (CurvilinearCoordinateSystem, optional): An optional curvilinear coordinate system. If None,
              a new one is created based on the reference path.

        Methods:
            evaluate_scenario(predictions, ego_pos, ego_orientation, ego_pos_cl, ego_v, timestep):
                Evaluates the current scenario by updating the internal state based on external inputs,
                calculating visible and occluded areas, identifying spawn points for phantom agents,
                and visualizing the scenario and predictions if enabled.
                Inputs:
                    predictions (dict): Predictions about other agents in the environment.
                    ego_pos (tuple): The current position of the ego vehicle.
                    ego_orientation (float): The current orientation of the ego vehicle in radians.
                    ego_pos_cl (tuple): The current position of the ego vehicle in curvilinear coordinates.
                    ego_v (float): The current velocity of the ego vehicle.
                    timestep (int): The current timestep of the simulation.
                Outputs:

        """

    def __init__(self, scenario, reference_path, vehicle_params, dt, cosy_cl=None):

        # load configuration from config file
        self.config = self._load_config()

        # load global variables that never change
        self.cr_scenario = scenario
        self.lanelet_network = scenario.lanelet_network
        self.ego_reference_path = reference_path
        self.cosy_cl = CurvilinearCoordinateSystem(reference_path) if cosy_cl is None else cosy_cl
        self.vehicle_params = vehicle_params
        self.dt = dt
        self.plot = self.config['plot']

        # initialize changing variables
        self.predictions = None
        self.ego_pos = None
        self.ego_orientation = None
        self.ego_pos_cl = None
        self.timestep = None
        self.spawn_points = []

        # define configuration
        self.sensor_radius = self.config['sensor_model']['sensor_radius']  # sensor radius in m
        self.sensor_angle = self.config['sensor_model']['sensor_angle']  # sensor angle

        # create visualization (mainly for debugging)
        self.visualization = FOVisualization(scenario, reference_path, self.plot)

        # convert obstacles to FOObstacles
        self.fo_obstacles = FOObstacles(self.cr_scenario.obstacles)

        # initialize sensor model
        self.sensor_model = SensorModel(lanelet_network=self.lanelet_network,
                                        ref_path=self.ego_reference_path,
                                        sensor_radius=self.sensor_radius,
                                        sensor_angle=self.sensor_angle,
                                        visualization=self.visualization,
                                        debug=True)

        # initialize agent_manager
        self.agent_manager = FOAgentManager(scenario=self.cr_scenario,
                                            reference_path=self.ego_reference_path,
                                            config=self.config['agent_manager'],
                                            visualization=self.visualization,
                                            timestep=self.timestep,
                                            dt=self.dt,
                                            debug=False,
                                            fo_obstacles=self.fo_obstacles)

        # initialize spawn locator
        self.spawn_locator = SpawnLocator(agent_manager=self.agent_manager,
                                          ref_path=self.ego_reference_path,
                                          config=self.config,
                                          cosy_cl=self.cosy_cl,
                                          sensor_model=self.sensor_model,
                                          fo_obstacles=self.fo_obstacles,
                                          visualization=self.visualization,
                                          debug=False)

        # initialize metrics
        self.metrics = Metric(self.config['metrics'], self.vehicle_params, self.agent_manager)

    def evaluate_scenario(self, predictions, ego_pos, ego_orientation, ego_pos_cl, ego_v, timestep):

        # update timestep
        self._update_time_step(timestep)

        # update variables with external inputs
        self.predictions = predictions
        self.ego_pos = ego_pos
        self.ego_orientation = ego_orientation
        self.ego_pos_cl = ego_pos_cl

        # reset agent manager
        self.agent_manager.reset()

        # clear spawn points
        self.spawn_points.clear()

        # visualize scenario if activated
        if self.visualization is not None and self.plot:
            self.visualization.draw_scenario(timestep=self.timestep)
            self.visualization.show_plot()

        # set relevant obstacles
        self.fo_obstacles.update(self.timestep)

        # calculate visible and occluded area
        self.sensor_model.calc_visible_and_occluded_area(timestep=self.timestep,
                                                         ego_pos=self.ego_pos,
                                                         ego_orientation=self.ego_orientation,
                                                         obstacles=self.fo_obstacles)

        # update multipolygon that stores the polygons with visible obstacles for spawn point detection
        self.fo_obstacles.update_multipolygon()

        # find spawn points
        self.spawn_points = self.spawn_locator.find_spawn_points(self.ego_pos, self.ego_orientation, self.ego_pos_cl, ego_v)

        # iterate over spawn points and add phantom agents
        for sp in self.spawn_points:
            # determine mode
            mode = 'lane_center' if sp.source == 'left turn' or sp.source == 'right turn' else 'ref_path'

            # add agent for each spawn point
            self.agent_manager.add_agent(pos=sp.position, velocity="default", agent_type=sp.agent_type,
                                         timestep=self.timestep, horizon=3.0, mode=mode, orientation=sp.orientation)

            # print debug message
            print("Phantom agent of type {} with id {} added to scenario at position {}"
                  .format(sp.agent_type, self.agent_manager.phantom_agents[-1].agent_id, sp.position))

        # update pedestrian predictions after adding real pedestrians to the scenario
        self.agent_manager.update_real_agents(self.predictions)

        # visualize PA predictions if activated
        if self.visualization is not None and self.plot:
            self.visualization.draw_predictions(self.agent_manager.predictions, label=False)
            self.visualization.show_plot(time=0.1)

        # return visible area if needed
        return self.sensor_model.visible_area

    def trajectory_safety_assessment(self, trajectory):
        metrics, safety_assessment = self.metrics.evaluate_metrics(trajectory)

        return metrics, safety_assessment

    def _update_time_step(self, timestep):
        # updates the timestep in all objects
        self.timestep = timestep
        self.sensor_model.timestep = timestep
        self.agent_manager.timestep = timestep

    @staticmethod
    def _load_config(filename='config.yaml'):
        # loads the config.yaml file
        try:
            file_path = os.path.join(os.path.dirname(__file__), "config", filename)
            # Attempt to open and parse the YAML file
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
            return data
        except FileNotFoundError:
            # This exception is raised if the file is not found
            print("The config file was not found.")
        except yaml.YAMLError as exc:
            # This exception is raised if there is an error parsing the YAML file
            print("An error occurred while parsing the YAML file.")
            if hasattr(exc, 'problem_mark'):
                mark = exc.problem_mark
                print(f"Error at position: ({mark.line + 1}, {mark.column + 1})")
        except Exception as exc:
            # General error case
            print(f"An unexpected error has occurred: {exc}")
