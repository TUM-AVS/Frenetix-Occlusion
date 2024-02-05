__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np


class TTCE:
    """
    Metric for Time to Closest Encounter (TTCE). Use DCE to calculate the TTCE value.
    DCE marks the time step when the minimal distance is reached.
    """

    def __init__(self, agent_manager):
        self.agent_manager = agent_manager

    def __repr__(self):
        return "<'Time to Closest Encounter Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    def evaluate(self, trajectory, results) -> dict:

        # check if required dce is available
        if 'dce' in results:

            # initialize dict
            ttce = {}

            for key in results['dce']:

                # calculate ttce for every obstacle
                ttce[key] = np.round(results['dce'][key]['time_dce'] * self.agent_manager.dt, 3)

            return ttce

        raise ValueError('DCE is not available in results, but is needed to evaluate TTCE metric!')
