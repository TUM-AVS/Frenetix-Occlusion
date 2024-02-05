__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np


class WTTC:
    """
    Metric for Worst Time to Collision (WTTC).
    The worst-time-to-collision metric extends the usual TTC by considering multiple traces of actors
    """

    def __init__(self):
        pass

    def __repr__(self):
        return "<'Worst Time to Collision  Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    @staticmethod
    def evaluate(trajectory, results) -> float:

        # init wttc
        wttc = np.inf

        # check if required dce is available
        if 'ttc' in results:

            for key in results['ttc']:

                # calculate wttc for the trajectory
                wttc = min(wttc, results['ttc'][key])

            return wttc

        raise ValueError('TTC is not available in results, but is needed to evaluate WTTC metric!')
