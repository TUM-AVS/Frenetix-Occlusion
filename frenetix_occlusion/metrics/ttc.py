__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np


class TTC:
    """
    Metric for Time to Collision (TTC). Calculate TTC with an intended trajectory of the ego vehicle and the given
    prediction of other vehicles. Use DCE to calculate the TTC value.
    """

    def __init__(self, agent_manager):
        self.agent_manager = agent_manager

    def __repr__(self):
        return "<'Distance to Closest Encounter Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    def evaluate(self, trajectory, results) -> dict:

        # check if required dce is available
        if 'dce' in results:

            # initialize dict
            ttc = {}

            for key in results['dce']:

                # cache variable and initialize ttc for each prediction
                dce = results['dce'][key]
                ttc[key] = np.inf

                # calculate ttc for every obstacle
                if np.isclose(dce['dce'], 0.0):

                    # calc ttc
                    ttc[key] = np.round(dce['time_dce'] * self.agent_manager.dt, 3)
            return ttc

        raise ValueError('DCE is not available in results, but is needed to evaluate TTC metric!')
