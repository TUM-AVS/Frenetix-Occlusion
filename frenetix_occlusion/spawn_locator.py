__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import copy
import numpy as np
from commonroad_dc.geometry.util import compute_pathlength_from_polyline, compute_curvature_from_polyline
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from commonroad_route_planner.utility.route import lanelet_orientation_at_position
import frenetix_occlusion.utils.helper_functions as hf


class SpawnPoint:
    """
    Simple class containing the spawn point information
    """
    def __init__(self, pos, agent_type, pos_cl=None, source=None, orientation=None):
        self.position = pos
        self.agent_type = agent_type
        self.cl_pos = pos_cl
        self.source = source
        self.orientation = orientation


class SpawnLocator:

    def __init__(self, agent_manager, ref_path, cosy_cl, sensor_model, fo_obstacles, config, visualization=None, debug=False):

        # load other modules
        self.agent_manager = agent_manager
        self.scenario = agent_manager.scenario
        self.sensor_model = sensor_model
        self.fo_obstacles = fo_obstacles
        self.visualization = visualization
        self.config = config['spawn_locator']
        self.ref_path = ref_path
        self.cosy_cl = cosy_cl

        # initialize own variables
        self.reference = None
        self.s = None
        self.reference_s = None
        self.ego_pos = None
        self.ego_orientation = None
        self.ego_cl = None
        self.spawn_points = []

        # define where to look for spawn points (from config)
        self.spawn_point_behind_turn = self.config['spawn_points_behind_turn']
        self.spawn_point_behind_dynamic_obstacle = self.config['spawn_point_behind_dynamic_obstacle']
        self.spawn_point_behind_static_obstacle = self.config['spawn_point_behind_static_obstacle']

        # maximum number of spawn points (from config)
        self.max_dynamic_spawn_points = self.config['max_dynamic_spawn_points']
        self.max_static_spawn_points = self.config['max_static_spawn_points']

        # initialize internal parameters
        self.ped_width = config['agent_manager']['pedestrian']['width']
        self.ped_length = config['agent_manager']['pedestrian']['length']
        self.s_threshold_time = 4  # needed for s_threshold --> calculates the distance when moving at current speed for X seconds
        self.min_s_threshold = 25  # min s_threshold, if vehicle is very slow
        self.s_threshold = None  # calculated by function - maximum allowed distance of PA
        self.tolerance_same_direction = np.radians(20)  # tolerance until a vehicle is defined as driving in same direction in °
        self.max_distance_to_other_obstacle = 30  # max allowed distance between ego vehicle and obstacle in m
        self.buffer_around_vehicle_from_side = 12  # buffer around vehicles in m
        self.min_area_threshold = 10  # min required area size in m² -> smaller areas are neglected
        self.agent_area_limits = {'Car': 9, 'Bicycle': 1.7}  # required size for a rectangle, that an agent can be spawned in m²
        self.min_distance_between_pedestrians = 5  # minimum distance between two pedestrians (behind static obstacles)
        self.debug = debug
        # spawn points behind turns (special parameters)
        self.offset_ref_path = {'left turn': 3, 'right turn': 0}
        self.phantom_offset_s = {'left turn': -0.5, 'right turn': 0}
        self.phantom_offset_d = {'left turn': 1, 'right turn': -1}

    def find_spawn_points(self, ego_pos, ego_orientation, ego_cl, ego_v):
        """
            Determines potential spawn points for phantom agents.

            This function calculates various spawn points for phantom agents considering the ego vehicle's current state
            and intended direction. It clears previous spawn points, updates internal state with the ego vehicle's
            current position and curvilinear coordinates, and calculates a threshold based on the vehicle's velocity.
            The function then determines the ego vehicle's intention (e.g., left turn, straight ahead) and searches for
            potential spawn points behind turns, dynamic obstacles, and static obstacles.

            Parameters:
            ego_pos (numpy.ndarray): The current position of the ego vehicle, as (x, y) coordinates.
            ego_cl (numpy.ndarray): The current curvilinear coordinates of the ego vehicle, as (s, d) coordinates.
            ego_v (float): The current velocity of the ego vehicle.

            Returns:
            list: A list of SpawnPoint objects, each representing a potential location for a phantom agent.
            The list may be empty if no suitable spawn points are found.

            Note:
            The function utilizes several internal properties (like `self.s_threshold_time`, `self.min_s_threshold`) and
            methods (like `_prepare_reference_path`, `_find_ego_intention`, `_find_spawn_point_behind_turn`, etc.) which
             are part of the class this function belongs to.
        """
        # clear previous points
        self.spawn_points.clear()

        # save values for other functions
        self.ego_pos = ego_pos
        self.ego_orientation = ego_orientation
        self.ego_cl = ego_cl

        # calculate threshold
        self.s_threshold = ego_cl[0] + max(ego_v * self.s_threshold_time, self.min_s_threshold)

        # calc new reference
        self.reference, self.reference_s = self._prepare_reference_path(ego_cl)

        # find ego intention within the new reference
        ego_intention = self._find_ego_intention(self.reference)

        if self.debug:
            print(ego_intention)

        # if ego vehicle wants to turn left or go straight, search possible spawn points behind dynamic obstacles
        if self.spawn_point_behind_dynamic_obstacle and (ego_intention == 'straight ahead' or ego_intention == 'left turn'):
            spawn_points_dynamic = self._find_spawn_point_behind_dynamic_obstacle()
            self._append_spawn_point(spawn_points_dynamic)

        # search possible spawn points behind static obstacles (e.g. parked cars)
        if self.spawn_point_behind_static_obstacle:
            spawn_point_static = self._find_spawn_point_behind_static_obstacle()
            self._append_spawn_point(spawn_point_static)

        # if ego vehicle wants to turn left or right, search possible spawn points behind turn
        if self.spawn_point_behind_turn and (ego_intention == 'left turn' or ego_intention == 'right turn'):
            spawn_point_turn = self._find_spawn_point_behind_turn(ego_intention)
            self._append_spawn_point(spawn_point_turn)

        return self.spawn_points

    #################################################################################
    #################### Spawn Points behind dynamic obstacle #######################
    #################################################################################

    def _find_spawn_point_behind_dynamic_obstacle(self) -> list[SpawnPoint] or None:
        """
            Finds spawn points behind dynamic obstacles that are currently visible and within a specific distance
            from the ego vehicle.

            The function goes through a series of steps:
            - Identifies visible dynamic obstacles.
            - Finds the current ego lanelet and relevant intersections.
            - Determines the occluded area behind these dynamic obstacles where a phantom agent could potentially spawn.
            - Computes the size and position of a 'phantom' box around the center of the occluded area.
            - Returns a list of suitable spawn points based on the computed 'phantom' boxes.

            Returns:
                list[SpawnPoint]: A list of SpawnPoint objects representing potential spawn locations behind dynamic obstacles.
                                  Each SpawnPoint contains the position, agent type, and other relevant data.
                                  Returns None if no suitable spawn point is found or if there are no visible dynamic obstacles.

            Note:
                The function uses several internal attributes of the class, such as `self.ego_pos`, `self.scenario`,
                and `self.sensor_model`, among others. It also uses helper functions for various calculations.
            """

        spawn_points = []

        # find currently visible dynamic obstacles
        visible_dyn_obst = [obst for obst in self.fo_obstacles
                            if obst.current_visible and obst.cr_obstacle.obstacle_role.name == 'DYNAMIC']

        # sort by distance
        visible_dyn_obst = sorted(visible_dyn_obst, key=lambda obstacle: np.linalg.norm(self.ego_pos - obstacle.current_pos))

        # quit if no visible obstacle is available
        if not visible_dyn_obst:
            return

        # find current ego lanelet
        ego_lanelet = self._find_lanelet_by_position(self.ego_pos)

        # find all intersections in scenario
        all_intersections = self.scenario.lanelet_network.intersections

        # find intersection of interest
        intersection, incoming_lanelets, intersection_lanelets = self._find_relevant_intersection(all_intersections, ego_lanelet)

        if intersection:

            # create a deepcopy for relevant lanelets
            relevant_lanelets = copy.deepcopy(incoming_lanelets)

            # combine both sets
            relevant_lanelets.update(intersection_lanelets)

            # remove ego lanelet
            relevant_lanelets.remove(ego_lanelet.lanelet_id)
        else:
            # no intersection -> relevant lanelets is other side of the road
            lanelets_along_reference_path = self._find_lanelets_along_reference(self.reference, step=5)
            relevant_lanelets = [lanelet.adj_left for lanelet in lanelets_along_reference_path]

        for dyn_obst in visible_dyn_obst:

            # continue if obstacle is pedestrian or bicycle
            if (dyn_obst.cr_obstacle.obstacle_type.value == 'bicycle' or
                    dyn_obst.cr_obstacle.obstacle_type.value == 'pedestrian'):
                continue

            # break if maximum number is reached
            if len(spawn_points) > self.max_dynamic_spawn_points:
                break

            # check distance between vehicles and continue, if to far away
            distance_to_obstacle = np.linalg.norm(self.ego_pos - dyn_obst.current_pos)
            if distance_to_obstacle > self.max_distance_to_other_obstacle:
                continue

            # find lanelets, where dynamic obstacle is located
            dyn_obst_lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([dyn_obst.current_pos])[0]

            # if dynamic obstacle is not on a relevant lanelet, skip obstacle
            if not any(element in relevant_lanelets for element in dyn_obst_lanelet_ids):
                continue

            # try to calculate curvilinear coordinates of obstacle [s, d] -> if outside projection domain continue
            try:
                dyn_obstacle_cl = self.cosy_cl.convert_to_curvilinear_coords(dyn_obst.current_pos[0], dyn_obst.current_pos[1])
            except:
                continue

            # if obstacle is behind ego, continue
            if dyn_obstacle_cl[0] < self.ego_cl[0] + 3 or abs(dyn_obstacle_cl[1]) > 15:
                continue

            if self.visualization is not None and self.debug:
                pass
                # self.visualization.plot_vector(pos=self.ego_pos, orientation=self.ego_orientation)
                # self.visualization.plot_vector(pos=dyn_obst.current_pos, orientation=dyn_obst.current_orientation)

            # find possible polygons where phantom agent is allowed and get their polygon
            dyn_obst_lanelets = [self.scenario.lanelet_network.find_lanelet_by_id(lnlt) for lnlt in dyn_obst_lanelet_ids
                                 if lnlt in relevant_lanelets]
            dyn_obst_lanelet_polygons = [lnlt.polygon.shapely_object for lnlt in dyn_obst_lanelets]

            # if all obstacle lanelets are in the intersection, add one predecessor for better area identification
            if intersection and all(lnlt_id in intersection_lanelets for lnlt_id in dyn_obst_lanelet_ids):
                predecessor_polygon = (self.scenario.lanelet_network.find_lanelet_by_id(dyn_obst_lanelets[0].predecessor[0]).
                                       polygon.shapely_object)
                dyn_obst_lanelet_polygons.append(predecessor_polygon)

            # combine polygon to possible polygon
            possible_polygon = unary_union(dyn_obst_lanelet_polygons)

            # calculate orientation difference between ego vehicle and dynamic obstacle
            orientation_diff = abs((dyn_obst.current_orientation - self.ego_orientation)) % (2 * np.pi)

            # if orientation is similar (+,- 20°) -> vehicles are driving in opposite directions
            if np.pi - self.tolerance_same_direction <= orientation_diff <= np.pi + self.tolerance_same_direction:
                if self.debug:
                    print('vehicles are on lanelets with opposite direction')
                relevant_occluded_area = (self.sensor_model.obstacle_occlusions[dyn_obst.cr_obstacle.obstacle_id].
                                          intersection(possible_polygon))
            else:
                if self.debug:
                    # dynamic obstacle is coming from other direction
                    print('obstacle is coming from other direction')

                # find relevant occluded area by cropping the possible polygon to the occluded area
                relevant_occluded_area = possible_polygon.intersection(self.sensor_model.occluded_area)

            # crop the relevant area to an area around the dynamic obstacle
            relevant_occluded_area = relevant_occluded_area.intersection(Point(dyn_obst.current_pos).buffer(self.buffer_around_vehicle_from_side))
            relevant_occluded_area = relevant_occluded_area.difference(dyn_obst.current_polygon.buffer(1))

            # only use part of the multipolygon with the largest area
            if relevant_occluded_area.geom_type == "MultiPolygon" or relevant_occluded_area.geom_type == "GeometryCollection":
                polygons = [geom for geom in relevant_occluded_area.geoms if isinstance(geom, Polygon)]
                relevant_occluded_area = max(polygons, key=lambda p: p.area)

            # check size of relevant occluded area and continue if it is too small
            if relevant_occluded_area.area < self.min_area_threshold:
                continue

            # convert centroid to numpy array
            center_pos = np.array([relevant_occluded_area.centroid.x, relevant_occluded_area.centroid.y])

            # find lanelets where centroid is located
            centroid_lanelet = (self.scenario.lanelet_network.find_lanelet_by_position([center_pos]))[0]

            # if centroid is not on a relevant lanelet, skip obstacle
            if not any(element in relevant_lanelets for element in centroid_lanelet):
                continue

            # if relevant_occluded_area is in front of the obstacle, continue
            [dx, dy] = hf.vector_from_angle(dyn_obst.current_orientation)
            if Point(dyn_obst.current_pos[0] + 4 * dx, dyn_obst.current_pos[1] + 4 * dy).within(relevant_occluded_area):
                continue

            # find size of box "phantom" box (5x2m) around center point of relevant occluded area
            rectangles = self._find_matching_rectangle(center_pos, relevant_occluded_area)

            for key in rectangles:
                polygon = rectangles[key]['polygon']
                metric = rectangles[key]['jaccard_similarity']
                if polygon.area >= self.agent_area_limits[key] and metric > 0.98:
                    phantom_pos = np.array([polygon.centroid.x, polygon.centroid.y])
                    spawn_points.append(SpawnPoint(pos=phantom_pos, agent_type=key, pos_cl=None, source='behind_dynamic_obstacle'))

            # draw debug plots if debug is turned on
            if self.visualization and self.debug:
                self.visualization.plot_poly_fast(relevant_occluded_area, color='blue', zorder=20)
                self.visualization.plot_poly_fast(rectangles['Car']['polygon'], color='k', zorder=20)
                self.visualization.plot_poly_fast(rectangles['Bicycle']['polygon'], color='r', zorder=20)

        return spawn_points

    #################################################################################
    #################### Spawn points behind static obstacle  #######################
    #################################################################################

    def _find_spawn_point_behind_static_obstacle(self) -> list[SpawnPoint] or None:
        """
        Identifies potential spawn points behind visible static obstacles within a specified distance from the ego vehicle.

        The function operates in several steps:
        - Identifies currently visible static obstacles and sorts them based on their distance to the ego vehicle.
        - Skips the process if no visible static obstacle is found.
        - Calculates curvilinear coordinates of each obstacle and determines if they are within a certain distance
          from the ego vehicle.
        - Converts the corner points of the obstacle to curvilinear coordinates and determines the minimum and
          maximum coordinates.
        - Creates lines perpendicular to the reference path to find intersections with the visible area.
        - Determines potential spawn positions based on the intersection of these lines with the visible area.
        - Ensures that these spawn positions do not intersect with other obstacles and are sufficiently spaced from existing
          spawn points.

        Returns:
            list[SpawnPoint]: A list of SpawnPoint objects representing potential spawn locations behind static obstacles.
                               Each SpawnPoint contains the position, agent type, curvilinear position, and source information.
                               Returns None if no suitable spawn point is found.

        Note:
            The function relies on various attributes of the class such as `self.ego_pos`, `self.cosy_cl`, ...
        """

        # initialize lists to store results
        spawn_points = []
        s_positions = []

        # find currently visible static obstacles
        visible_stat_obst = [obst for obst in self.fo_obstacles
                             if obst.current_visible and obst.cr_obstacle.obstacle_role.name == 'STATIC']

        visible_stat_obst = sorted(visible_stat_obst, key=lambda obstacle: np.linalg.norm(self.ego_pos - obstacle.current_pos))

        # quit if no visible obstacle is available
        if not visible_stat_obst:
            return

        for stat_obst in visible_stat_obst:

            # break if maximum number is reached
            if len(spawn_points) > self.max_static_spawn_points:
                break

            # check distance between vehicles and continue, if to far away
            distance_to_obstacle = np.linalg.norm(self.ego_pos - stat_obst.current_pos)
            if distance_to_obstacle > self.max_distance_to_other_obstacle:
                continue

            # try to calculate curvilinear coordinates of obstacle [s, d] -> if outside projection domain continue
            try:
                stat_obstacle_cl = self.cosy_cl.convert_to_curvilinear_coords(stat_obst.current_pos[0], stat_obst.current_pos[1])
            except:
                continue

            # if obstacle is behind ego or too far away, continue
            if self.ego_cl[0] + self.s_threshold < stat_obstacle_cl[0] or stat_obstacle_cl[0] < self.ego_cl[0] + 3:
                continue

            # convert corner points of vehicle to curvilinear coordinates
            list_of_corner_points = [np.array([[x], [y]]) for x, y in stat_obst.current_corner_points]
            list_of_corner_points_cl = np.array(self.cosy_cl.convert_list_of_points_to_curvilinear_coords(list_of_corner_points, 4))

            # find minimum and maximum curvilinear coordinates of obstacle ( + s offset)
            offset = 0.8
            s_min, s_max = np.min(list_of_corner_points_cl[:, 0]) - offset, np.max(list_of_corner_points_cl[:, 0]) + offset
            d_min, d_max = np.min(list_of_corner_points_cl[:, 1]) - offset, np.max(list_of_corner_points_cl[:, 1]) + offset

            # create lines perpendicular to reference path to find intersection with visible area [front, back of obstacle]
            lines_cl = [np.array([[s_min, d_min], [s_min, d_max]]), np.array([[s_max, d_min], [s_max, d_max]])]

            for line_cl in lines_cl:
                try:
                    # try to convert lines to cartesian coordinates -> failes sometimes due to the curvilinear cosy
                    line = np.array([self.cosy_cl.convert_to_cartesian_coords(p[0], p[1]) for p in line_cl])
                except:
                    continue

                # convert to LineString
                line_ls = LineString(line)

                # if possible path does not intersect with visible area and occluded area --> continue
                if not line_ls.intersects(self.sensor_model.occluded_area) or not line_ls.intersects(self.sensor_model.visible_area):
                    continue

                # if buffered line intersects with another visible obstacle
                if line_ls.buffer(self.ped_width/2).intersects(self.fo_obstacles.visible_obstacle_multipolygon):
                    continue

                # find intersection with visible area -> this marks the possible spawn position
                spawn_pos_point = self.sensor_model.visible_area.buffer(self.ped_length / 2 * 1.3). \
                    exterior.intersection(line_ls)

                # check if calculated point is a multipoint --> find point closest to the left lanelet vertices and within
                # occluded area
                if spawn_pos_point.geom_type == 'MultiPoint':
                    # select one point of lanelet
                    obst_lanelet = self._find_lanelet_by_position(stat_obst.current_pos)
                    left_vertices_point = obst_lanelet.left_vertices[0]

                    # sort points
                    points = [point for point in spawn_pos_point.geoms]
                    points = sorted(points, key=lambda point: np.linalg.norm(left_vertices_point - [point.x, point.y]))

                    # iterate over points and
                    spawn_pos_point = None
                    for point in points:
                        if point.within(self.sensor_model.occluded_area):
                            spawn_pos_point = point
                            break

                # if no spawn pos point could be found, continue
                if spawn_pos_point is None:
                    continue

                # if buffered position intersects with visible area --> is visible --> continue
                if spawn_pos_point.buffer(0.15).intersects(self.sensor_model.visible_area):
                    continue

                # if spawn point buffer is not within the road polygon
                if not spawn_pos_point.buffer(0.15).within(self.sensor_model.road_polygon):
                    continue

                # buffer variables
                spawn_pos = np.array([spawn_pos_point.x, spawn_pos_point.y])
                spawn_pos_cl = self.cosy_cl.convert_to_curvilinear_coords(spawn_pos[0], spawn_pos[1])
                source = 'behind static obstacle ' + str(stat_obst.cr_obstacle.obstacle_id)

                # check longitudinal distance between new spawn point and already added spawn points
                if any(abs(s - spawn_pos_cl[0]) <= self.min_distance_between_pedestrians for s in s_positions):
                    continue

                # once the code reaches this spot, a suitable spawn point is found

                # find orientation of lanelet at obstacle position --> pedestrian orientation is orientation + 90°
                _, orientation = self._find_orientation_at_position(stat_obst.current_pos)
                orientation = orientation + np.pi/2

                # append spawn point and break for loop --> only one spawn point per obstacle
                spawn_points.append(SpawnPoint(pos=spawn_pos, agent_type="Pedestrian", pos_cl=spawn_pos_cl,
                                               source=source, orientation=orientation))

                # save curvilinear spawn points
                s_positions.append(spawn_pos_cl[0])

                # debug visualization
                if self.visualization and self.debug:
                    self.visualization.rnd.ax.plot(line[:, 0], line[:, 1], 'r', zorder=200)
                    self.visualization.rnd.ax.plot(spawn_pos[0], spawn_pos[1], 'ro', zorder=200)

                break

        return spawn_points

    #################################################################################
    ######################## Spawn Points behind turns ##############################
    #################################################################################
    def _find_spawn_point_behind_turn(self, ego_intention) -> SpawnPoint or None:
        """
            Finds a spawn point for a phantom agent behind a corner based on the ego vehicle's intention and the
            current environment configuration.

            This function calculates a potential spawn point for a pedestrian (phantom agent) by analyzing
            the intersection of the ego vehicle's reference path with occluded areas detected by its sensor model.
            It considers the ego vehicle's intention (e.g., left turn) and uses various offsets and thresholds to
            determine the most suitable spawn point. The function also checks if the suggested spawn point is within a
            visible area and adjusts it if necessary.

            Parameters:
            ego_intention (str): The intended direction of the ego vehicle ('left turn', 'right turn').
            This determines how the reference path is manipulated and used for spawn point calculation.

            Returns:
            SpawnPoint or None: A SpawnPoint object containing the position and other details of the phantom agent if a
            suitable location is found. Returns None if no suitable spawn point is identified or if the suggested point
            does not meet certain criteria (e.g., not within visible area, not within another obstacle).

            Raises:
            ValueError: If an unknown intersection type is encountered during the process.

            Note:
            This function uses several internal properties and methods (`self.reference`, `self.sensor_model`, etc.)
            which are part of the class this function belongs to.
            """

        # convert ref path to linestring to find intersection
        if ego_intention == 'left turn':
            # if ego is turning left, the ref path is shifted (offset) to the other
            # side of the road for spawn point identification
            curvilinear_list = np.column_stack((self.reference_s,
                                                np.full(self.reference_s.shape, self.offset_ref_path['left turn'])))
            ref_path_parallel = self._convert_curvilinear_list_to_cartesian_coordinates(curvilinear_list)
            ref_path_ls = LineString(ref_path_parallel)
        else:
            ref_path_ls = LineString(self.reference)

        # find intersection of linestring and occluded area
        occluded_area_intersection = ref_path_ls.intersection(self.sensor_model.occluded_area)

        # check intersection type and find first intersection point
        if occluded_area_intersection.is_empty:
            return
        elif occluded_area_intersection.geom_type == 'LineString':
            intersection = np.array(occluded_area_intersection.coords)[0]
        elif occluded_area_intersection.geom_type == 'MultiLineString':
            intersection = np.array(occluded_area_intersection.geoms[-1].coords)[0]
            if self.debug:
                print('MultiLineString in spawn point processing detected')
        else:
            raise ValueError('Unknown intersection type!')

        # find s coordinate of first intersection
        s_intersection = self.cosy_cl.convert_to_curvilinear_coords(intersection[0], intersection[1])[0]

        # calculate s coordinate of possible phantom agent crossing point (with offset)
        s_phantom = s_intersection + self.phantom_offset_s[ego_intention]

        # end here if s_phantom is too far away (> s_threshold) or behind ego
        if s_phantom > self.s_threshold or s_phantom < self.ego_cl[0] + 3:
            return

        # calculate d offset of desired point
        d_offset = self.phantom_offset_d[ego_intention] + self.offset_ref_path[ego_intention]

        # convert s_pedestrian to cartesian coordinates for further checks
        phantom_pos = self.cosy_cl.convert_to_cartesian_coords(s_phantom, d_offset)

        # check, if position os within visible area and shift if necessary
        while Point(phantom_pos).buffer(0.5).intersects(self.sensor_model.visible_area):
            s_phantom = s_phantom + 0.5
            phantom_pos = self.cosy_cl.convert_to_cartesian_coords(s_phantom, d_offset)

        # if suggested point is within another obstacle, return None
        if Point(phantom_pos).buffer(0.5).intersects(self.fo_obstacles.visible_obstacle_multipolygon):
            return

        # find ego lanelet id, lanelet and orientation
        ego_lanelet, ego_lanelet_orientation = self._find_orientation_at_position(self.ego_pos)

        # find lanelet id, lanelet and orientation at possible pedestrian point
        phantom_lanelet, phantom_lanelet_orientation = self._find_orientation_at_position(phantom_pos)

        # calculate orientation difference of both lanelets
        orientation_diff = abs((phantom_lanelet_orientation - ego_lanelet_orientation)) % (2 * np.pi)

        # check orientation difference against threshold (45°) -> if orientation is too similar,
        # the spawn point is not behind the turn -> return
        if orientation_diff < np.radians(45):
            return

        # create spawn point and return it
        spawn_point = SpawnPoint(pos=phantom_pos, agent_type="Pedestrian",
                                 pos_cl=[s_phantom, self.phantom_offset_d[ego_intention]], source=ego_intention)

        return spawn_point

    #################################################################################
    ############################### Helper functions ################################
    #################################################################################
    @staticmethod
    def _find_intersection_lanelets(intersection) -> tuple[set[int], set[int]]:
        """
        Find all incoming lanelets and intersecting lanelets of intersection
        """
        incoming_lanelets = set()
        intersection_lanelets = set()

        for intersection_element in intersection.incomings:
            incoming_lanelets.update(intersection_element.incoming_lanelets)
            intersection_lanelets.update(intersection_element.successors_left)
            intersection_lanelets.update(intersection_element.successors_right)
            intersection_lanelets.update(intersection_element.successors_straight)

        return incoming_lanelets, intersection_lanelets

    def _find_intersection_outgoings(self, intersection_incomings) -> list:
        """
        Find all outgoing lanelets of intersection by finding all adjacent lefts of incoming lanelets
        """
        outgoings = []
        for lanelet_id in intersection_incomings:
            lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            outgoings.append(lanelet.adj_left)

        return outgoings

    def _find_relevant_intersection(self, intersections, ego_lanelet):
        """
        Find relevant intersection (the one the ego vehicles approaches next) from list of intersections
        """
        for intersection in intersections:
            # find incoming lanelets and intersection lanelets
            incoming_lanelets, intersection_lanelets = self._find_intersection_lanelets(intersection)

            # check if ego lanelet leads toward intersection or is within intersection
            if ego_lanelet.lanelet_id in incoming_lanelets or ego_lanelet.lanelet_id in intersection_lanelets:
                return intersection, incoming_lanelets, intersection_lanelets

        return None, None, None

    def _find_lanelets_along_reference(self, reference, step=5) -> list:
        """
        Find lanelets along reference path
        """
        lanelets = []
        for i in range(0, len(reference), step):
            point = reference[i]
            lanelet = self._find_lanelet_by_position(point)
            if lanelet is not None and lanelet not in lanelets:
                lanelets.append(lanelet)

        return lanelets

    def _append_spawn_point(self, spawn_point):
        """
        Simple helper function to append spawn point to list self.spawn_points if it is not None
        """
        if spawn_point is not None:
            if type(spawn_point) is list:
                self.spawn_points.extend([point for point in spawn_point if point is not None])
            else:
                self.spawn_points.append(spawn_point)

    def _convert_curvilinear_list_to_cartesian_coordinates(self, curvilinear_list):
        """
        Converts a numpy array with curvilinear points to a numpy array with cartesian points given the current cosy
        """
        cartesian_list = []
        for item in curvilinear_list:
            cartesian_list.append(self.cosy_cl.convert_to_cartesian_coords(item[0], item[1]))

        return np.array(cartesian_list)

    def _find_orientation_at_position(self, pos):
        """
        Find lanelet id, lanelet and orientation at given point
        """
        lanelet = self._find_lanelet_by_position(pos)
        lanelet_orientation = lanelet_orientation_at_position(lanelet, pos)

        return lanelet, lanelet_orientation

    def _find_lanelet_by_position(self, pos):
        """
        Find lanelet at given point
        """
        lanelet_id = self.scenario.lanelet_network.find_lanelet_by_position([pos])
        if not lanelet_id[0]:
            return None
        lanelet_id = lanelet_id[0][0]
        lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)

        return lanelet

    def _prepare_reference_path(self, ego_cl, distance=40):
        """
        Cut reference path to "length" of interest
        """
        # compute path length of reference path
        self.s = compute_pathlength_from_polyline(self.ref_path)

        # find start and end index of reference path
        index_start = self._find_nearest_index(self.s, ego_cl[0])
        index_end = self._find_nearest_index(self.s, ego_cl[0] + distance)

        # cut reference path to interesting area
        reference = self.ref_path[index_start:index_end]
        reference_s = self.s[index_start:index_end]

        return reference, reference_s

    def _find_matching_rectangle(self, position, allowed_area):
        # calculate orientation at center pos
        _, orientation = self._find_orientation_at_position(position)

        # create orientated polygon
        vehicle_oriented_rectangle = hf.create_oriented_rectangle(position, 5.5, 2.5, orientation)
        poly_vehicle = vehicle_oriented_rectangle.intersection(allowed_area)
        vehicle = self._calculate_polygon_metrics(poly_vehicle)
        vehicle['polygon'] = poly_vehicle

        bike_oriented_rectangle = hf.create_oriented_rectangle([poly_vehicle.centroid.x, poly_vehicle.centroid.y], 2, 1, orientation)
        poly_bike = bike_oriented_rectangle.intersection(allowed_area)
        bike = self._calculate_polygon_metrics(poly_bike)
        bike['polygon'] = poly_bike

        return {"Car": vehicle, "Bicycle": bike}

    @staticmethod
    def _calculate_polygon_metrics(polygon):

        # Minimum Rotated Rectangle for the given polygon
        mbr = polygon.minimum_rotated_rectangle

        # aspect ratio
        area_ratio = polygon.area / mbr.area

        # Jaccard-similarity
        intersection = polygon.intersection(mbr).area
        union = unary_union([polygon, mbr]).area
        jaccard_similarity = intersection / union

        return {"area_ratio": area_ratio, "jaccard_similarity": jaccard_similarity}

    @staticmethod
    def _find_ego_intention(reference):
        """
        Find the ego intention of the vehicle within the reference line by analyzing the curvature
        """
        # Calculate curvature of the reference path
        curvature = compute_curvature_from_polyline(reference)
        # Determine the vehicle intention
        if max(curvature) > 0.10:  # approximately 60 degrees in radians
            return "left turn"
        elif min(curvature) < -0.10:
            return "right turn"
        else:
            return "straight ahead"

    @staticmethod
    def _find_nearest_index(path_s, current_s):
        """
        Find the index in the path that is closest to the current s-coordinate.
        """
        # Calculate the difference between each s-coordinate in the path and the current s-coordinate
        differences = np.abs(path_s - current_s)
        # Find the index of the smallest difference
        nearest_index = np.argmin(differences)
        return nearest_index
