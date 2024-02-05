__author__ = "Maximilian Geisslinger, Rainer Trauth, Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np
from commonroad.scenario.obstacle import ObstacleType
from frenetix_occlusion.metrics.utils.logistic_regression import (get_protected_inj_prob_log_reg_ignore_angle,
                                                                  get_protected_inj_prob_log_reg_reduced_sym)

# Dictionary for existence of protective crash structure.
obstacle_protection = {
    ObstacleType.CAR: True,
    ObstacleType.TRUCK: True,
    ObstacleType.BUS: True,
    ObstacleType.BICYCLE: False,
    ObstacleType.PEDESTRIAN: False,
    ObstacleType.PRIORITY_VEHICLE: True,
    ObstacleType.PARKED_VEHICLE: True,
    ObstacleType.TRAIN: True,
    ObstacleType.MOTORCYCLE: False,
    ObstacleType.TAXI: True,
    ObstacleType.ROAD_BOUNDARY: None,
    ObstacleType.PILLAR: None,
    ObstacleType.CONSTRUCTION_ZONE: None,
    ObstacleType.BUILDING: None,
    ObstacleType.MEDIAN_STRIP: None,
    ObstacleType.UNKNOWN: False,
}


def get_harm(agent_manager, traj, predictions, vehicle_params, modes, coeffs):
    """Get harm.

    Args:
        scenario (_type_): _description_
        traj (_type_): _description_
        predictions (_type_): _description_
        ego_vehicle_type (_type_): _description_
        vehicle_params (_type_): _description_
        modes (_type_): _description_
        coeffs (_type_): _description_
        timer (_type_): _description_

    Returns:
        _type_: _description_
    """
    # get the IDs of the predicted obstacles
    obstacle_ids = list(predictions.keys())

    # initialize dicts
    ego_harm_traj = {}
    obst_harm_traj = {}

    for obstacle_id in obstacle_ids:
        # get agent from agent manager by its prediction id
        agent = agent_manager.agent_by_prediction_id(obstacle_id)
        agent_type = agent.agent_type.lower()

        # choose which model should be used to calculate the harm
        ego_harm_fun, obstacle_harm_fun = get_model(modes, agent_type)

        # only calculate the risk as long as both obstacles are in the scenario
        pred_path = predictions[obstacle_id]['pos_list']
        pred_length = min(len(traj.cartesian.x) - 1, len(pred_path))
        if pred_length == 0:
            continue

        # get the size, the velocity and the orientation of the predicted vehicle
        pred_size = (predictions[obstacle_id]['shape']['length'] * predictions[obstacle_id]['shape']['width'])
        pred_v = np.array(predictions[obstacle_id]['v_list'], dtype=np.float64)
        pred_yaw = np.array(predictions[obstacle_id]['orientation_list'], dtype=np.float64)

        # get the predicted obstacle vehicle mass
        obstacle_mass = get_obstacle_mass(obstacle_type=ObstacleType(agent_type), size=pred_size)

        # crash angle between ego vehicle and considered obstacle [rad]
        pdof_array = predictions[obstacle_id]["orientation_list"][:pred_length] - traj.cartesian.theta[:pred_length] + np.pi
        rel_angle_array = np.arctan2(predictions[obstacle_id]["pos_list"][:pred_length, 1] - traj.cartesian.y[:pred_length],
                                      predictions[obstacle_id]["pos_list"][:pred_length, 0] - traj.cartesian.x[:pred_length])

        # angle of impact area for the ego vehicle
        ego_angle_array = rel_angle_array - traj.cartesian.theta[:pred_length]
        # angle of impact area for the obstacle
        obs_angle_array = np.pi + rel_angle_array - predictions[obstacle_id]["orientation_list"][:pred_length]

        # calculate the difference between pre-crash and post-crash speed
        delta_v_array = np.sqrt(
            np.power(traj.cartesian.v[:pred_length], 2)
            + np.power(pred_v[:pred_length], 2)
            + 2 * traj.cartesian.v[:pred_length] * pred_v[:pred_length] * np.cos(pdof_array)
        )
        ego_delta_v = obstacle_mass / (vehicle_params.mass + obstacle_mass) * delta_v_array
        obstacle_delta_v = vehicle_params.mass / (vehicle_params.mass + obstacle_mass) * delta_v_array

        # calculate harm based on selected model
        ego_harm_obst = ego_harm_fun(velocity=ego_delta_v, angle=ego_angle_array, coeff=coeffs)
        obst_harm_obst = obstacle_harm_fun(velocity=obstacle_delta_v, angle=obs_angle_array, coeff=coeffs)

        # store harm list for the obstacles in dictionary for current frenét trajectory
        ego_harm_traj[obstacle_id] = ego_harm_obst
        obst_harm_traj[obstacle_id] = obst_harm_obst

    return ego_harm_traj, obst_harm_traj

def get_model(modes, agent_type):
    """Get harm model according to settings.

    Args:
        modes (_type_): _description_


    Raises:
        ValueError: _description_

    Returns:
        _type_: _description_
    """
    # obstacle protection type
    obs_protection = obstacle_protection[ObstacleType(agent_type)]

    if modes["harm_mode"] == "log_reg":
        # select case based on protection structure
        if obs_protection is True:
            # use log reg sym reduced
            # calculate harm for the ego vehicle
            ego_harm = get_protected_inj_prob_log_reg_reduced_sym

            # calculate harm for the obstacle vehicle
            obstacle_harm = get_protected_inj_prob_log_reg_reduced_sym

        elif obs_protection is False:
            # calc ego harm
            ego_harm = get_protected_inj_prob_log_reg_ignore_angle

            # calculate obstacle harm
            # logistic regression model
            obstacle_harm = lambda velocity,angle,coeff : 1 / (  # noqa E731
                1
                + np.exp(
                    coeff["pedestrian"]["const"]
                    - coeff["pedestrian"]["speed"] * velocity
                )
            )
        else:
            ego_harm = lambda velocity,angle,coeff : 1  # noqa E731
            obstacle_harm = lambda velocity,angle,coeff : 1  # noqa E731

    else:
        raise ValueError("Please select a valid mode for harm estimation (log_reg)")

    return ego_harm, obstacle_harm


def get_obstacle_mass(obstacle_type, size):
    """
    Get the mass of the considered obstacle.

    Args:
        obstacle_type (ObstacleType): Type of considered obstacle.
        size (float): Size (length * width) of the vehicle in m².

    Returns:
        Mass (float): Estimated mass of considered obstacle.
    """
    if obstacle_type == ObstacleType.CAR:
        return -1333.5 + 526.9 * np.power(size, 0.8)
    elif obstacle_type == ObstacleType.TRUCK:
        return 25000
    elif obstacle_type == ObstacleType.BUS:
        return 13000
    elif obstacle_type == ObstacleType.BICYCLE:
        return 90
    elif obstacle_type == ObstacleType.PEDESTRIAN:
        return 75
    elif obstacle_type == ObstacleType.PRIORITY_VEHICLE:
        return -1333.5 + 526.9 * np.power(size, 0.8)
    elif obstacle_type == ObstacleType.PARKED_VEHICLE:
        return -1333.5 + 526.9 * np.power(size, 0.8)
    elif obstacle_type == ObstacleType.TRAIN:
        return 118800
    elif obstacle_type == ObstacleType.MOTORCYCLE:
        return 250
    elif obstacle_type == ObstacleType.TAXI:
        return -1333.5 + 526.9 * np.power(size, 0.8)
    else:
        return 0
