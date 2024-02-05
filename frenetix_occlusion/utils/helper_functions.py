__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
from shapely.geometry import Polygon, Point, MultiPolygon, LineString
from shapely.affinity import rotate, translate
import numpy as np


def create_oriented_rectangle(pos, length, width, orientation):
    """
    Creates an oriented rectangle in Shapely with a given centroid, width, height, and angle.

    :param pos: numpy.ndarray with x and y coordinates
    :param length: Length of the rectangle
    :param width: Width of the rectangle
    :param orientation: Angle of orientation in radiant, counterclockwise from the x-axis
    :return: A Shapely Polygon object representing the rectangle
    """
    # Create a rectangle centered at the origin
    dx = length / 2.0
    dy = width / 2.0
    rectangle = Polygon([(-dx, -dy), (-dx, dy), (dx, dy), (dx, -dy)])

    # Rotate the rectangle around the origin
    rotated_rectangle = rotate(rectangle, np.degrees(orientation), origin=(0, 0), use_radians=False)

    # Translate the rectangle so that the centroid is at the specified point
    oriented_rectangle = translate(rotated_rectangle, pos[0], pos[1])

    return oriented_rectangle


def calc_normal_vector_to_curve(curve, pos):
    """
        Calculates the normal vector on a linestring at a given point
        Args:
            curve: curve as numpy array
            pos: numpy 1D array with x and y coordinates of the point

        Returns:
            normal vector towards curve at given point
        """
    # convert curve into shapely linestring
    linestring = LineString(curve)
    point = Point(pos)

    # find the closest point on linestring
    closest_point_on_ls = np.array(linestring.interpolate(linestring.project(point)).coords).flatten()

    # calc vector using the points
    normal_vector = _unit_vector(closest_point_on_ls - pos)

    return normal_vector


def angle_between_positive(v1, v2):
    """Returns the positive angle (mathematically positive) in radians between vectors 'v1' and 'v2':"""
    v1_u = _unit_vector(v1)
    v2_u = _unit_vector(v2)
    dot_product = np.dot(v1_u, v2_u)
    angle_rad = np.arctan2(np.linalg.det([v1_u, v2_u]), dot_product)
    if angle_rad < 0:
        angle_rad += 2 * np.pi
    return angle_rad


def vector_from_angle(angle_rad):
    """ Calculates vector from given angle"""
    x = np.cos(angle_rad)
    y = np.sin(angle_rad)
    return np.array([x, y])


def create_polygon_from_vertices(vert_point1, vert_point2, ego_pos):
    """Creates a polygon for the area that is occluded from two vertice points.

    Arguments:
        vert_point1 {[list]} -- [x,y of first point of object]
        vert_point2 {[list]} -- [x,y of second point of object]
        ego_pos {[list]} -- [x,y of ego position]

    Returns:
        poly [Shapely polygon] -- [Represents the occluded area]
    """

    pol_point1 = vert_point1 + 100 * (vert_point1 - ego_pos)
    pol_point2 = vert_point2 + 100 * (vert_point2 - ego_pos)

    poly = Polygon([vert_point1, vert_point2, pol_point2, pol_point1, vert_point1])

    return poly


def calc_corner_points(pos, orientation, obstacle_shape):
    """Calculate corner points of a dynamic obstacles in global coordinate system.

    Arguments:
        pos:  position of the object (center position) in global coordinate system [x,y] - np.array
        orientation: orientation of the object in rad - float
        obstacle_shape: shape of the object [width and length]

    Returns:
        corner points of object
    """
    corner_points = _rotate_point_by_angle(obstacle_shape.vertices[0:4], orientation)
    corner_points = [p + pos for p in corner_points]
    return np.array(corner_points)


def remove_unwanted_shapely_elements(polys) -> MultiPolygon:
    """
    This function removes every Geometry except Polygons from a GeometryCollection
    and converts the remaining Polygons to a MultiPolygon.

    Args:
        polys: GeometryCollection

    Returns: MultiPolygon

    """
    if polys.geom_type == 'Polygon':
        return polys

    poly_list = []
    for pol in polys.geoms:
        if pol.geom_type == 'Polygon' and not np.isclose(pol.area, 0):
            poly_list.append(pol)

    multipolygon = MultiPolygon(poly_list)

    return multipolygon.buffer(0)


def get_polygon_from_obstacle_occlusion(ego_pos, corner_points):

    # Identify points for geometric projection
    c1, c2 = _identify_projection_points(ego_pos, corner_points)

    # Create polygon with points far away in the ray direction of ego pos
    c3 = c2 + _unit_vector(c2 - ego_pos) * 100
    c4 = c1 + _unit_vector(c1 - ego_pos) * 100

    occlusion = Polygon([c1, c2, c3, c4])
    return occlusion, c1, c2

def _identify_projection_points(ego_pos, corner_points):
    """This function identifies the two points of a rectangular object that are the edges from an ego pos point of view.

    Arguments:
        corner_points {[type]} -- [description]
        ego_pos {[type]} -- [description]

    Returns:
        [type] -- [description]
    """

    max_angle = 0

    for edge_point1 in corner_points:
        for edge_point2 in corner_points:
            ray1 = edge_point1 - ego_pos
            ray2 = edge_point2 - ego_pos

            angle = _angle_between(ray1, ray2)

            if angle > max_angle:
                max_angle = angle
                ret_edge_point1 = edge_point1
                ret_edge_point2 = edge_point2

    return ret_edge_point1, ret_edge_point2


def _rotate_point_by_angle(point, angle):
    """Rotate any point by an angle.

    Arguments:
        point:
        angle:
        point {[type]} -- [description]
        angle {[type]} -- [description]

    Returns:
        [type] -- [description]
    """
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.matmul(rotation_matrix, point.transpose()).transpose()


def _unit_vector(vector):
    """Returns the unit vector of the vector."""
    return vector / np.linalg.norm(vector)


def _angle_between(v1, v2):
    """Returns the angle in radians between vectors 'v1' and 'v2':"""
    v1_u = _unit_vector(v1)
    v2_u = _unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


