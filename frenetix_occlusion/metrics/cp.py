__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
from frenetix_occlusion.metrics.utils.collision_probability import get_collision_probability


class CP:
    def __init__(self, vehicle_params, agent_manager):
        self.vehicle_params = vehicle_params
        self.agent_manager = agent_manager

    # overwrite function description for debugging
    def __repr__(self):
        return "<'Collision Probability Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    def evaluate(self, trajectory, results) -> dict:
        """
        Calculate the collision probability for the given trajectory.

        Args:
            trajectory (FrenetTrajectory): Considered frenét trajectory.

        Returns:
            dict: Dictionary with collision probability
        """
        # set predictions
        predictions = self.agent_manager.predictions

        # calculate collision probability based on the collision model
        coll_prob_dict = get_collision_probability(traj=trajectory, predictions=predictions,
                                                   vehicle_params=self.vehicle_params)

        return coll_prob_dict
