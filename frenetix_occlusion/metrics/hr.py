__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import os
import json
import numpy as np
from frenetix_occlusion.metrics.utils.harm_model import get_harm


class HR:
    """
    Metric for Harm and Risk (HR).
    """
    # initialize parameters
    def __init__(self, vehicle_params, agent_manager):
        self.risk_params = self._load_param("risk_params")
        self.harm_params = self._load_param("harm_params")
        self.vehicle_params = vehicle_params
        self.agent_manager = agent_manager

    # overwrite function description for debugging
    def __repr__(self):
        return "<'Risk and Harm Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    # helper function to load parameters
    @staticmethod
    def _load_param(name):
        file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", name + ".json")
        with open(file_path, 'r') as file:
            data = json.load(file)
            return data

    # evaluates the harm and risk metric
    def evaluate(self, trajectory, result) -> dict:
        """
        Calculate the risk for the given trajectory.

        Args:
            trajectory (FrenetTrajectory): Considered frenét trajectory.

        Returns:
            dict: Dictionary with ego and obstacle risks for every time step concerning every obstacle
        """

        modes = self.risk_params
        coeffs = self.harm_params

        # set predictions
        predictions = self.agent_manager.predictions

        # calculate collision probability based on the collision model
        coll_prob_dict = result['cp']

        # calculate ego and obstacle harm for each obstacle (no collision probability considered)
        ego_harm_traj, obst_harm_traj = get_harm(agent_manager=self.agent_manager, traj=trajectory, predictions=predictions,
                                                 vehicle_params=self.vehicle_params, modes=modes, coeffs=coeffs)

        # initialize variables
        harm_risk = {}
        max_ego_risk_all = 0
        max_obst_risk_all = 0
        max_ego_harm_all = 0
        max_obst_harm_all = 0
        max_collision_probability_all = 0
        max_obst_harm_with_cp_all = 0

        for key in ego_harm_traj:
            # calculate risk with harm and collision probability
            ego_risk_traj = [ego_harm_traj[key][t] * coll_prob_dict[key][t] for t in range(len(ego_harm_traj[key]))]
            obst_risk_traj = [obst_harm_traj[key][t] * coll_prob_dict[key][t] for t in range(len(obst_harm_traj[key]))]

            if np.max(coll_prob_dict[key]) > 0.01:
                harm_with_cp = obst_harm_traj[key][np.argmax(coll_prob_dict[key])]
            else:
                harm_with_cp = 0.0

            # save values in dict
            harm_risk[key] = {'max_ego_risk': max(ego_risk_traj),
                              'max_obst_risk': max(obst_risk_traj),
                              'max_obst_harm_with_cp': harm_with_cp,
                              'max_obst_risk_index': obst_risk_traj.index(max(obst_risk_traj)),
                              'max_ego_harm': max(ego_harm_traj[key]),
                              'max_obst_harm': max(obst_harm_traj[key]),
                              'ego_risk_traj': ego_risk_traj,
                              'obst_risk_traj': obst_risk_traj,
                              'ego_harm_traj': ego_harm_traj[key],
                              'obst_harm_traj': obst_harm_traj[key],
                              'collision_probability': coll_prob_dict[key],
                              'max_collision_probability': max(coll_prob_dict[key])}

            # find maximum values of all obstacles
            max_ego_risk_all = max(max_ego_risk_all, harm_risk[key]['max_ego_risk'])
            max_obst_risk_all = max(max_obst_risk_all, harm_risk[key]['max_obst_risk'])
            max_ego_harm_all = max(max_ego_harm_all, harm_risk[key]['max_ego_harm'])
            max_obst_harm_all = max(max_obst_harm_all, harm_risk[key]['max_obst_harm'])
            max_obst_harm_with_cp_all = max(max_obst_harm_with_cp_all, harm_risk[key]['max_obst_harm_with_cp'])
            max_collision_probability_all = max(max_collision_probability_all, harm_risk[key]['max_collision_probability'])

        # save the highest global values in dict
        harm_risk['max_ego_risk_all'] = max_ego_risk_all
        harm_risk['max_obst_risk_all'] = max_obst_risk_all
        harm_risk['max_ego_harm_all'] = max_ego_harm_all
        harm_risk['max_obst_harm_all'] = max_obst_harm_all
        harm_risk['max_collision_probability_all'] = max_collision_probability_all
        harm_risk['max_obst_harm_with_cp_all'] = max_obst_harm_with_cp_all

        return harm_risk
