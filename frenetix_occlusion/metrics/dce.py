__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np
from frenetix_occlusion.metrics.utils.convert_dynamic_obstacle import (convert_traj_to_dyn_obstacle,
                                                                       convert_prediction_to_dyn_obstacle)


class DCE:
    """
    Metric for Distance to Closest Encounter (DCE).
    """

    def __init__(self, vehicle_params, agent_manager):
        self.vehicle_params = vehicle_params
        self.agent_manager = agent_manager

    def __repr__(self):
        return "<'Distance to Closest Encounter Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    def evaluate(self, trajectory, results) -> dict:
        """
        Computes the shortest distance of ego-vehicle and obstacle by calculating the distance between both polygons
        (boundaries of both objects) for each time step and returning the minimum.
        """

        # initialize variables
        result = {}
        predictions = self.agent_manager.predictions

        # convert ego trajectory to dynamic obstacle
        ego_dyn_obstacle = convert_traj_to_dyn_obstacle(trajectory, self.vehicle_params)

        # convert all predictions to dynamic obstacle
        for key in self.agent_manager.predictions:
            agent = self.agent_manager.agent_by_prediction_id(key)
            prediction_dyn_obstacle = convert_prediction_to_dyn_obstacle(agent, key, predictions[key])
            dce, time_dce = self._calc_dce(ego_dyn_obstacle, prediction_dyn_obstacle)
            result[key] = {'dce': dce, 'time_dce': time_dce}

        return result

    def _calc_dce(self, ego_dyn_obstacle, prediction_dyn_obstacle, debug=False) -> tuple[float, int]:
        """
        Calculates the Distance to Closest Encounter between the ego vehicle and the predicted obstacle.
        The code is adapted from Commonroad CriMe (https://commonroad.in.tum.de/tools/commonroad-crime) - DCE
        :param ego_dyn_obstacle: dynamic obstacle of ego vehicle
        :param prediction_dyn_obstacle: dynamic obstacle of prediction
        :return: (float): dce
        """

        # Cache attributes for quicker access
        state_list = ego_dyn_obstacle.prediction.trajectory.state_list

        # set dce to inf
        dce = np.inf
        time_dce = 0

        # iterate over all time steps
        for i in range(0, len(state_list)):

            # check if predicted obstacle exists at timestep
            if prediction_dyn_obstacle.occupancy_at_time(i) is not None:

                # load polygons from prediction
                ego_poly = ego_dyn_obstacle.occupancy_at_time(i).shape.shapely_object
                other_poly = prediction_dyn_obstacle.occupancy_at_time(i).shape.shapely_object

                # calculate distance between objects
                distance = np.round(ego_poly.distance(other_poly), 3)

                # check if distance is smaller than current dce and save it if needed
                if distance < dce:
                    time_dce = i
                    dce = distance
                if dce == 0.0:
                    break
            else:
                break

            # debug
            if debug is True and self.agent_manager.visualization is not None:
                mpl1 = self.agent_manager.visualization.plot_poly_fast(ego_poly, color='k', zorder=50)
                mpl2 = self.agent_manager.visualization.plot_poly_fast(other_poly, color='r', zorder=50)
                self.agent_manager.visualization.show_plot(time=0.1)
                print(dce)
                mpl1.remove()
                mpl2.remove()

        return dce, time_dce
