__author__ = "Korbinian Moller,"
__copyright__ = "TUM AVS"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
from functools import reduce
import numpy as np
from shapely.geometry import Polygon, Point
import frenetix_occlusion.utils.helper_functions as hf
from shapely.geometry.multipolygon import MultiPolygon


class SensorModel:
    def __init__(self, lanelet_network, ref_path, sensor_radius=30, sensor_angle=90, debug=True, visualization=None):

        # load global variables that never change
        self.lanelet_network = lanelet_network
        self.road_polygon, self.lanelet_network_polygons = self._convert_lanelet_network()  # combined lanelets as a shapely polygon and polygon list
        self.ref_path = ref_path
        self.visualization = visualization

        # initialize changing variables
        self.sensor_sector = None
        self.visible_area = None
        self.occluded_area = None
        self.all_obstacle_occlusions_polygon = None
        self.obstacle_occlusions = {}
        self.visible_objects_timestep = None
        self.timestep = None
        self.ego_pos = None
        self.ego_orientation = None

        # define configuration
        self.sensor_radius = sensor_radius
        self.sensor_angle = sensor_angle
        self.debug = debug

    def calc_visible_and_occluded_area(self, timestep, ego_pos, ego_orientation, obstacles):

        # Set variables
        self.ego_pos = ego_pos
        self.ego_orientation = ego_orientation
        self.visible_objects_timestep = []

        # reset obstacle occlusions list
        self.obstacle_occlusions.clear()

        # calc visible area #
        # calculate visible area based on lanelet geometry
        visible_area_road = self._calc_visible_area_from_lanelet_geometry()

        # calculate visible area based on obstacles
        visible_area, self.all_obstacle_occlusions_polygon = self._calc_visible_area_from_obstacle_occlusions(visible_area_road, obstacles)

        # get visible obstacles and add to list (has to be done after finishing the calculation of the visible area)
        visible_area_check = visible_area.buffer(0.01, join_style=2)

        if visible_area_check.is_valid is False:
            visible_area_check = visible_area.buffer(0.01)

        for obst in obstacles:

            # if obstacle exists at the current timestep
            if obst.current_pos is not None:

                # check if obstacle intersects with visible area
                if obst.current_polygon.intersects(visible_area_check):
                    # add to list of visible objects
                    self.visible_objects_timestep.append(obst.cr_obstacle.obstacle_id)

                    # update obstacle
                    obst.current_visible = True
                    obst.last_visible_at_ts = timestep

        # remove linestrings from visible_area
        self.visible_area = hf.remove_unwanted_shapely_elements(visible_area)

        # calc occluded area #

        # find the sector of interest
        # calculate two points with 180-degree angle
        angle_start = self.ego_orientation - np.radians(90)
        angle_end = self.ego_orientation + np.radians(90)
        sector = self._calc_relevant_sector(angle_start, angle_end, factor=1.5)

        # Find the relevant area --> lanelet along reference path and within the sector
        relevant_area_poly = sector.intersection(self.road_polygon)

        # calc occluded area
        self.occluded_area = relevant_area_poly.difference(self.visible_area)

        # debug visualization
        if self.debug and self.visualization:
            self.visualization.draw_point(ego_pos)
            self.visualization.fill_area(self.visible_area, color='g')
            self.visualization.fill_area(self.occluded_area, color='r')

        return self.visible_area

    def _calc_visible_area_from_lanelet_geometry(self):
        """
        Calculates the visible area at the ego position in consideration of the lanelet network geometry

        Returns:
        visible area
        """

        # at first, the visible area is the entire road polygon
        visible_area = self.road_polygon

        # calculate two points based on the sensor opening angle
        angle_start = self.ego_orientation - np.radians(self.sensor_angle / 2)
        angle_end = self.ego_orientation + np.radians(self.sensor_angle / 2)

        # create a "sector" with the given angles points and the ego position
        if self.sensor_angle >= 359.9:
            self.sensor_sector = Point(self.ego_pos).buffer(self.sensor_radius)
        else:
            self.sensor_sector = self._calc_relevant_sector(angle_start, angle_end)

        # find the intersection between the "circle" and the "triangle"
        visible_area = visible_area.intersection(self.sensor_sector)

        # remove unwanted elements
        visible_area = self._remove_unwanted_shapely_elements(visible_area)

        # Subtract areas that can not be seen due to geometry
        if visible_area.geom_type == 'MultiPolygon':
            points_vis_area = np.concatenate([np.array(p.exterior.xy).T for p in visible_area.geoms])
        else:
            points_vis_area = np.array(visible_area.exterior.xy).T

        # find points of vertices and create points far away
        for idx in range(points_vis_area.shape[0] - 1):
            vert_point1 = points_vis_area[idx]
            vert_point2 = points_vis_area[idx + 1]

            # create polygon from vertices
            pol = hf.create_polygon_from_vertices(vert_point1, vert_point2, self.ego_pos)

            # subtract polygon from visible area if it is valid
            if pol.is_valid:
                area_check = visible_area.difference(pol)
                # shapely has a bug, that visible area can be empty after performing .difference for no reason
                if area_check.is_empty:
                    # a very small buffer fixes that
                    visible_area = visible_area.buffer(0.0001).difference(pol)
                else:
                    visible_area = area_check

            # remove unwanted elements
            visible_area = self._remove_unwanted_shapely_elements(visible_area)

        return visible_area

    def _calc_visible_area_from_obstacle_occlusions(self, visible_area, obstacles):
        """
        Calculate occlusions from obstacles and subtract them from visible_area
        Args:
            visible_area: visible area
            obstacles: list of obstacles of type FOObstacle

        Returns:
        updated visible area
        multipolygon of obstacles
        """

        obstacles_polygon = Polygon([])

        # Calculate occlusions from obstacles and subtract them from visible_area
        for obst in obstacles:

            # obstacle position is not empty, this happens if dynamic obstacle is not available at timestep
            if obst.current_pos is not None and obst.cr_obstacle.obstacle_type.value != 'bicycle':

                # check if within sensor radius or if obstacle intersects with visible area
                if obst.current_pos_point.within(visible_area) or obst.current_polygon.intersects(visible_area):
                    # calculate occlusion polygon that is caused by the obstacle
                    occlusion, c1, c2 = hf.get_polygon_from_obstacle_occlusion(self.ego_pos, obst.current_corner_points)
                    self.obstacle_occlusions[obst.cr_obstacle.obstacle_id] = occlusion.difference(obst.current_polygon)

                    # Subtract obstacle shape from visible area
                    visible_area = visible_area.difference(obst.current_polygon.buffer(0.005, join_style=2))
                    obstacles_polygon = obstacles_polygon.union(obst.current_polygon)

                    # Subtract occlusion caused by obstacle (everything behind obstacle) from visible area
                    if occlusion.is_valid:
                        visible_area = visible_area.difference(occlusion)

        return visible_area, obstacles_polygon

    def _convert_lanelet_network(self):
        lanelet_polygons = [polygon.shapely_object for polygon in self.lanelet_network.lanelet_polygons]
        road_polygon = reduce(lambda x, y: x.union(y), lanelet_polygons)

        return road_polygon, lanelet_polygons

    def _calc_relevant_sector(self, angle_start, angle_end, factor=1.0):
        points = [(self.ego_pos[0] + self.sensor_radius * factor * np.cos(angle),
                   self.ego_pos[1] + self.sensor_radius * factor * np.sin(angle))
                  for angle in np.linspace(angle_start, angle_end, 100)]

        # create a "sector" with these points and the ego position
        sector = Polygon([self.ego_pos] + points + [self.ego_pos])

        return sector

    @staticmethod
    def _remove_unwanted_shapely_elements(polys) -> MultiPolygon:
        """
        This function removes every Geometry except Polygons from a GeometryCollection
        and converts the remaining Polygons to a MultiPolygon.

        Args:
            polys: GeometryCollection

        Returns: MultiPolygon

        """
        if polys.geom_type == "GeometryCollection":
            poly_list = []
            for pol in polys.geoms:
                if pol.geom_type == 'Polygon':
                    if not np.isclose(pol.area, 0, 1.e-3):
                        poly_list.append(pol)

            multipolygon = MultiPolygon(poly_list)
        else:
            multipolygon = polys

        return multipolygon.buffer(0)
