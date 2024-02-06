__author__ = "Korbinian Moller,"
__copyright__ = "TUM AVS"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
from commonroad_route_planner.route_planner import Route
from commonroad_route_planner.utility.route import lanelet_orientation_at_position
from commonroad_route_planner.route import RouteType
from typing import List


class FORoutePlanner:
    def __init__(self, scenario, lanelet_network, visualization, debug):
        # global variables - never change
        self.cr_scenario = scenario
        self.lanelet_network = lanelet_network
        self.debug = debug
        self.visualization = visualization

        # changing variables
        self.lanelet_orientation = None
        self.start_lanelet = None
        self.route_candidates = None
        self.reference_paths = None

    def calc_possible_reference_paths(self, pos):

        # find current lanelet id and lanelet
        start_lanelet_id = self.cr_scenario.lanelet_network.find_lanelet_by_position([pos])[0][0]
        start_lanelet = self.cr_scenario.lanelet_network.find_lanelet_by_id(start_lanelet_id)

        # find lanelet orientation at initial position
        self.lanelet_orientation = lanelet_orientation_at_position(start_lanelet, pos)

        # calculate all routes using recursive function
        all_routes = self._find_all_routes(start_lanelet_id, max_depth=2)

        # convert "graph ids" to real route using commonroad Route class
        self.route_candidates: List = [Route(self.lanelet_network, route, route_type=RouteType.REGULAR) for route in all_routes if route]

        # store reference path separately
        self.reference_paths: List = [route.reference_path for route in self.route_candidates]

        if self.debug is True and self.visualization is not None: #  ToDo remove when not needed anymore
            self.visualization.draw_point(pos, color='k', zorder=20)
            for cand in self.route_candidates:
                self.visualization.draw_reference_path(cand.reference_path, zorder=20)

        return self.reference_paths

    def _find_all_routes(self, id_lanelet_start, max_depth=2):
        all_routes = []
        self._explore_routes(id_lanelet_start, [], all_routes, 0, max_depth)
        if not all_routes:
            raise ValueError('[OAP - Route Planner] Route Explorer could not find a Route')
        return all_routes

    def _explore_routes(self, id_lanelet_current, route, all_routes, depth, max_depth):
        lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet_current)

        # Add current lanelet to the route
        route.append(lanelet.lanelet_id)

        successors = []
        if lanelet.successor:
            successors.extend(lanelet.successor)
        if lanelet.adj_right and lanelet.adj_right_same_direction:
            lanelet_adj_right = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
            if lanelet_adj_right.successor:
                successors.append(lanelet.adj_right)
        if lanelet.adj_left and lanelet.adj_left_same_direction:
            lanelet_adj_left = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
            if lanelet_adj_left.successor:
                successors.append(lanelet.adj_left)

        if depth >= max_depth:
            successors = []
            # Max depth reached, return without exploring further

        if not successors:
            # If no successors, save route and return
            all_routes.append(route.copy())
            return

        for successor in successors:
            self._explore_routes(successor, route, all_routes, depth + 1, max_depth)
            route.pop()  # Backtrack to explore other paths



