__author__ = "Maximilian Geisslinger, Rainer Trauth, Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import numpy as np
import commonroad_dc.pycrcc as pycrcc
from scipy.stats import mvn


def get_collision_probability(traj, predictions: dict, vehicle_params):
    """
    Calculate the collision probabilities of a trajectory and predictions.

    Args:
        traj (FrenetTrajectory): Considered trajectory.
        predictions (dict): Predictions of the visible obstacles.
        vehicle_params (VehicleParameters): Parameters of the considered
            vehicle.

    Returns:
        dict: Collision probability of the trajectory per time step with the
            prediction for every visible obstacle.
    """
    obstacle_ids = list(predictions.keys())
    collision_prob_dict = {}

    # get the current positions array of the ego vehicles
    ego_pos = np.stack((traj.cartesian.x, traj.cartesian.y), axis=-1)

    # offset between vehicle center point and corner point
    offset = np.array([vehicle_params.length / 6, vehicle_params.width / 2])

    for obstacle_id in obstacle_ids:
        mean_list = predictions[obstacle_id]['pos_list']
        cov_list = predictions[obstacle_id]['cov_list']
        yaw_list = predictions[obstacle_id]['orientation_list']
        length = predictions[obstacle_id]['shape']['length']
        probs = []

        # mean distance calculation
        # determine the length of arrays
        min_len = min(len(traj.cartesian.x), len(mean_list))

        # adjust array of the ego vehicles
        ego_pos_array = ego_pos[1:min_len]

        # get the positions array of the front and the back of the obstacle vehicle
        mean_deviation_array = np.stack((np.cos(yaw_list[1:min_len]), np.sin(yaw_list[1:min_len])), axis=-1) * length / 2
        mean_array = np.array(mean_list[:min_len - 1])
        mean_front_array = mean_array + mean_deviation_array
        mean_back_array = mean_array - mean_deviation_array

        # total_mean_array = np.stack((mean_array, mean_front_array, mean_back_array))
        total_mean_array = np.array([mean_array, mean_front_array, mean_back_array])

        # distance from ego vehicle
        distance_array = total_mean_array - ego_pos_array
        distance_array = np.sqrt(distance_array[:, :, 0] ** 2 + distance_array[:, :, 1] ** 2)

        # min distance of each column
        min_distance_array = distance_array.min(axis=0)
        # bool: whether min distance is larger than 5.0
        min_distance_array = min_distance_array > 5.0

        for i in range(1, len(traj.cartesian.x)):
            # only calculate probability as the predicted obstacle is visible
            if i < len(mean_list):
                # if the distance between the vehicles is bigger than 5 meters,
                # the collision probability is zero
                # avoids huge computation times

                # directly use previous bool result for the if statements
                if (min_distance_array[i - 1]):
                    prob = 0.0
                else:
                    cov = cov_list[i - 1]
                    # if the covariance is a zero matrix, the prediction is
                    # derived from the ground truth
                    # a zero matrix is not singular and therefore no valid
                    # covariance matrix
                    allcovs = [cov[0][0], cov[0][1], cov[1][0], cov[1][1]]
                    if all(covi == 0 for covi in allcovs):
                        cov = [[0.1, 0.0], [0.0, 0.1]]

                    prob = 0.0

                    # the occupancy of the ego vehicle is approximated by three
                    # axis aligned rectangles
                    # get the center points of these three rectangles
                    center_points = get_center_points_for_shape_estimation(
                        length=vehicle_params.length,
                        width=vehicle_params.width,
                        orientation=traj.cartesian.theta[i],
                        pos=ego_pos_array[i - 1],
                    )

                    # upper_right and lower_left points
                    center_points = np.array(center_points)

                    # in order to get the cdf, the upper right point and the
                    # lower left point of every rectangle is needed
                    upper_right = center_points + offset
                    lower_left = center_points - offset

                    # use mvn.mvnun to calculate multivariant cdf
                    # the probability distribution consists of the partial
                    # multivariate normal distributions
                    # this allows to consider the length of the predicted
                    # obstacle
                    # consider every distribution
                    for mu in total_mean_array[:, i - 1]:
                        for center_point_index in range(len(center_points)):
                            prob += mvn.mvnun(lower_left[center_point_index], upper_right[center_point_index], mu, cov)[0]
            else:
                prob = 0.0
            # divide by 3 since 3 probability distributions are added up and
            # normalize the probability
            probs.append(prob / 3)

        collision_prob_dict[obstacle_id] = np.array(probs)

    return collision_prob_dict


def get_center_points_for_shape_estimation(
    length: float, width: float, orientation: float, pos: np.array
):
    """
    Get the 3 center points for axis aligned rectangles.

    Get the 3 center points for axis aligned rectangles that approximate an
    orientated rectangle.

    Args:
        length (float): Length of the oriented rectangle.
        width (float): Width of the oriented rectangle.
        orientation (float): Orientation of the oriented rectangle.
        pos (np.array): Center of the oriented rectangle.

    Returns:
        [np.array]: Array with 3 entries, every entry holds the center of one
            axis aligned rectangle.
    """
    # create the oriented rectangle
    obj = pycrcc.RectOBB(length / 2, width / 2, orientation, pos[0], pos[1])

    center_points = []
    obj_center = obj.center()
    # get the directional vector
    r_x = obj.r_x()
    # get the length
    a_x = obj.local_x_axis()
    # append three points (center point of the rectangle, center point of the
    # front third of the rectangle and center point of the back third of the
    # rectangle)
    center_points.append(obj_center)
    center_points.append(obj_center + r_x * (2 / 3) * a_x)
    center_points.append(obj_center - r_x * (2 / 3) * a_x)

    return center_points
