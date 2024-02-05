__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
import frenetix_occlusion.utils.helper_functions as hf
from shapely.geometry import Point, Polygon
from shapely.geometry.multipolygon import MultiPolygon


class FOObstacles:
    def __init__(self, cr_obstacles):
        self.cr_obstacles = cr_obstacles
        self.fo_obstacles = [FOObstacle(obst) for obst in cr_obstacles]
        self.visible_obstacle_multipolygon = None

    def __iter__(self):
        self.current = 0
        return self

    def __next__(self):
        # called when using the loop
        if self.current < len(self.fo_obstacles):

            # returns an element
            obstacle = self.fo_obstacles[self.current]
            self.current += 1
            return obstacle
        else:
            # stop when no more elements are available
            raise StopIteration

    def add(self, cr_obstacle):
        # add commonroad obstacle to FOObstacles manually (e.g. after creating new commonroad obstacle)
        self.cr_obstacles.append(cr_obstacle)
        self.fo_obstacles.append(FOObstacle(cr_obstacle))

    def update(self, timestep):
        for obst in self.fo_obstacles:
            obst.update_at_timestep(timestep)

    def update_multipolygon(self):
        poly_list = [obst.current_polygon for obst in self.fo_obstacles if obst.current_visible]
        self.visible_obstacle_multipolygon = MultiPolygon(poly_list)


class FOObstacle:
    def __init__(self, obst):

        # global variables - never changing
        self.cr_obstacle = obst
        self.initial_timestep = obst.initial_state.time_step

        # changing variables - will be available after calling update_at_timestep
        self.global_timestep = None
        self.relative_time_step = None
        self.current_pos = None
        self.current_pos_point = None
        self.current_orientation = None
        self.current_corner_points = None
        self.current_polygon = None
        self.current_visible = False
        self.last_visible_at_ts = None

        if self.cr_obstacle.obstacle_role.name == "STATIC":
            self._get_values_from_initial_state()

    def __repr__(self):
        return "<'Occlusion Obstacle with ID {}': {}.{} object at {}>".format(
            self.cr_obstacle.obstacle_id,
            self.__class__.__module__,
            self.__class__.__name__,
            hex(id(self))
        )

    def update_at_timestep(self, timestep):
        self.global_timestep = timestep
        self.relative_time_step = timestep - self.initial_timestep
        self.current_visible = False

        if self.cr_obstacle.obstacle_role.name == "DYNAMIC":
            if self.relative_time_step == 0:
                self._get_values_from_initial_state()

            elif self.relative_time_step >= 1:
                idx = self.relative_time_step - 1
                if idx < len(self.cr_obstacle.prediction.trajectory.state_list):
                    self._get_values_from_trajectory_prediction(idx)
                else:
                    self._set_all_values_to_none()

    def _get_values_from_initial_state(self):
        self.current_pos = self.cr_obstacle.initial_state.position
        self.current_pos_point = Point(self.current_pos)
        self.current_orientation = self.cr_obstacle.initial_state.orientation
        self.current_corner_points = hf.calc_corner_points(self.current_pos, self.current_orientation,
                                                           self.cr_obstacle.obstacle_shape)
        self.current_polygon = Polygon(self.current_corner_points)

    def _get_values_from_trajectory_prediction(self, idx):
        self.current_pos = self.cr_obstacle.prediction.trajectory.state_list[idx].position
        self.current_orientation = self.cr_obstacle.prediction.trajectory.state_list[idx].orientation
        self.current_corner_points = hf.calc_corner_points(self.current_pos, self.current_orientation,
                                                           self.cr_obstacle.obstacle_shape)
        self.current_polygon = Polygon(self.current_corner_points)
        self.current_pos_point = Point(self.current_pos)

    def _set_all_values_to_none(self):
        self.current_pos = None
        self.current_orientation = None
        self.current_corner_points = None
        self.current_polygon = None
        self.current_pos_point = None
