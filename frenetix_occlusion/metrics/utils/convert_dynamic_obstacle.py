__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import CustomState


def convert_prediction_to_dyn_obstacle(agent, key, prediction) -> DynamicObstacle:
    """
    Converts the given Prediction  to a DynamicObstacle
    :param agent: agent
    :param key: id for the new dynamic obstacle
    :param prediction: the prediction to convert
    :return: (DynamicObstacle)
    """

    # Precompute values that are static or can be vectorized
    positions = prediction['pos_list']
    time_steps = np.arange(len(positions))
    orientations = prediction['orientation_list']
    velocities = prediction['v_list']

    # create commonroad states (with conversion from rear axle to center point reference)
    state_list = [CustomState(time_step=int(time_step),  # Convert numpy int64 to Python int
                              position=position,
                              orientation=orientation,
                              velocity=velocity)
                  for time_step, position, orientation, velocity in
                  zip(time_steps, positions, orientations, velocities)]

    # create shape for trajectory occupancy
    shape = agent.shape

    # get trajectory prediction
    prediction = TrajectoryPrediction(Trajectory(initial_time_step=0, state_list=state_list), shape)

    # convert prediction to dynamic obstacle
    dynamic_obstacle = DynamicObstacle(key, agent.obstacle_type, shape, state_list[0], prediction)

    return dynamic_obstacle


def convert_traj_to_dyn_obstacle(trajectory, vehicle_params, obstacle_id: int = 42) -> DynamicObstacle:
    """
    Converts the given Frenet-Trajectory (reference point on rear axle) to a DynamicObstacle
    :param trajectory: the trajectory to check
    :return: (DynamicObstacle)
    """
    # Cache attributes for quicker access
    cartesian = trajectory.cartesian
    wb_r_a = vehicle_params.wb_rear_axle

    # Precompute values that are static or can be vectorized
    time_steps = np.arange(len(cartesian.x))
    positions = np.vstack((cartesian.x, cartesian.y)).T
    orientations = cartesian.theta
    velocities = cartesian.v

    # create commonroad states (with conversion from rear axle to center point reference)
    state_list = [CustomState(time_step=int(time_step),  # Convert numpy int64 to Python int
                              position=position,
                              orientation=orientation,
                              velocity=velocity)
                  .translate_rotate(np.array([wb_r_a * np.cos(orientation), wb_r_a * np.sin(orientation)]), 0.0)
                  for time_step, position, orientation, velocity in
                  zip(time_steps, positions, orientations, velocities)]

    # create shape for trajectory occupancy
    shape = Rectangle(vehicle_params.length, vehicle_params.width)

    # get trajectory prediction
    prediction = TrajectoryPrediction(Trajectory(initial_time_step=0, state_list=state_list), shape)

    # convert prediction to dynamic obstacle
    dynamic_obstacle = DynamicObstacle(obstacle_id, ObstacleType.CAR, shape, state_list[0], prediction)

    return dynamic_obstacle
