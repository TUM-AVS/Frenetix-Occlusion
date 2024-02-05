__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np
from scipy.interpolate import interp1d
import copy
from frenetix_occlusion.metrics.utils.convert_dynamic_obstacle import convert_traj_to_dyn_obstacle, convert_prediction_to_dyn_obstacle


class BE:
    """
    Metric for Break Evaluation (BE). Use DCE to calculate the BE value.
    """

    def __init__(self, vehicle_params, agent_manager):
        self.vehicle_params = vehicle_params
        self.agent_manager = agent_manager

    def __repr__(self):
        return "<'Break Evaluation Metric': {}.{} object at {}>".format(
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    def evaluate(self, trajectory, results) -> dict:

        # check if required dce is available
        if 'dce' in results:

            # initialize dict
            be = {}

            for key in results['ttc']:

                # cache variable
                ttc = results['ttc'][key]

                # init variables
                required_constant_deceleration = 0.0
                break_threat_number = 0.0

                # check if vehicle crashes
                if ttc is not np.inf:
                    if ttc > 0:

                        # calc the minimum required constant deceleration using the bisection method
                        required_constant_deceleration = self.find_minimum_deceleration(trajectory, key)

                        # calculate the break threat number
                        break_threat_number = required_constant_deceleration / self.vehicle_params.a_max

                # save results in dict
                be[key] = {'required_constant_deceleration': required_constant_deceleration,
                           'break_threat_number': break_threat_number}

            return be

        raise ValueError('DCE is not available in results, but is needed to evaluate BE metric!')

    def find_minimum_deceleration(self, traj, key, min_decel=0, max_decel=5, threshold=0.1, max_iterations=10):
        # use the bisection method to find the minimum required deceleration
        min_decel = np.round(abs(min(min(traj.cartesian.a), min_decel)), 2)
        for _ in range(max_iterations):
            current_decel = (min_decel + max_decel) / 2
            deceleration_trajectories, _ = self._calc_deceleration_trajectory(traj, deceleration=[current_decel])
            collisions = self._collision_check(deceleration_trajectories, key, self.agent_manager.predictions[key])

            if 0 in collisions:  # no collision
                max_decel = current_decel
            else:  # collision
                min_decel = current_decel

            if max_decel - min_decel < threshold:
                break

        return current_decel

    def _calc_deceleration_trajectory(self, traj, break_time_step=1, deceleration=None):
        if deceleration is None:
            deceleration = [0, 3]
        dt = self.agent_manager.dt
        trajectories = []

        x = traj.cartesian.x
        y = traj.cartesian.y
        theta = traj.cartesian.theta
        a = traj.cartesian.a
        v = traj.cartesian.v

        # Number of time steps
        num_steps = len(x)

        # Compute the traveled distances after each timestep
        dist = np.insert(np.cumsum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)), 0, 0)

        # calc new velocity vector based on initial velocity and deceleration
        time = np.arange(num_steps - break_time_step) * dt

        # create deceleration vector
        if len(deceleration) > 1:
            deceleration = np.arange(deceleration[0], deceleration[1], 0.1)

        for decel in deceleration:
            v_new = np.insert(np.maximum(v[break_time_step] - decel * time, 0), 0, v[0:break_time_step])
            a_new = np.concatenate((a[:break_time_step], np.ones(num_steps - break_time_step) * (- decel)))

            # calc the new traveled distance based on the new velocity vector
            dist_new = np.insert(np.cumsum(v_new * dt), 0, 0)[:-1]

            # create interpolation functions
            interp_theta = interp1d(dist, theta, kind='linear')
            interp_x = interp1d(dist, x, kind='linear')
            interp_y = interp1d(dist, y, kind='linear')

            # interpolate x,y and theta
            x_new = interp_x(dist_new)
            y_new = interp_y(dist_new)
            theta_new = interp_theta(dist_new)

            decel_traj = copy.deepcopy(traj)

            # save new values in trajectory dict
            decel_traj.cartesian.x = x_new
            decel_traj.cartesian.y = y_new
            decel_traj.cartesian.v = v_new
            decel_traj.cartesian.theta = theta_new
            decel_traj.cartesian.a = a_new

            # clear other values from trajectory
            decel_traj.costMap = {}
            decel_traj.feasabilityMap = {}
            decel_traj.cost = 0
            decel_traj.feasible = False
            decel_traj.sampling_parameters = np.empty(0)
            decel_traj.uniqueId = 0
            decel_traj.valid = False

            trajectories.append(decel_traj)

        return trajectories, deceleration

    def _collision_check(self, trajectories, key, prediction, debug=False):

        agent = self.agent_manager.agent_by_prediction_id(key)
        dyn_obst = convert_prediction_to_dyn_obstacle(agent, key, prediction)

        # Initialize a list with -1 indicating trajectories yet to be checked
        collisions = [-1 for _ in range(len(trajectories))]

        for traj_index, traj in enumerate(trajectories):
            # Convert trajectory to dynamic obstacle
            ego_obst = convert_traj_to_dyn_obstacle(traj, self.vehicle_params)
            state_list = ego_obst.prediction.trajectory.state_list

            # Assume no collision initially for this trajectory
            collision_detected = False

            # Iterate over all time steps in the trajectory
            for i in range(len(state_list)):
                # Check if predicted obstacle exists at this time step
                if dyn_obst.occupancy_at_time(i) is not None:
                    # Load polygons from prediction
                    ego_poly = ego_obst.occupancy_at_time(i).shape.shapely_object
                    other_poly = dyn_obst.occupancy_at_time(i).shape.shapely_object

                    # debug
                    if debug is True and self.agent_manager.visualization is not None:
                        mpl1 = self.agent_manager.visualization.plot_poly_fast(ego_poly, color='k', zorder=50)
                        mpl2 = self.agent_manager.visualization.plot_poly_fast(other_poly, color='r', zorder=50)
                        self.agent_manager.visualization.show_plot(time=0.2)
                        mpl1.remove()
                        mpl2.remove()

                    # Check for collision
                    if ego_poly.intersects(other_poly):
                        collisions[traj_index] = 1  # Collision detected
                        collision_detected = True
                        break
                    else:
                        collisions[traj_index] = 0  # No collision at this time step

            # If no collision detected for this trajectory, return the result
            if not collision_detected:
                return collisions

            # Return collisions list in case all trajectories are checked
        return collisions
