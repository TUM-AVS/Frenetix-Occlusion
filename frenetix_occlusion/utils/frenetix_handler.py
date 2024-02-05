__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports

# general
import numpy as np

# frenetix
import frenetix
import frenetix.trajectory_functions
import frenetix.trajectory_functions.feasability_functions as ff
from frenetix_occlusion.utils.sampling import generate_sampling_matrix, SamplingHandler
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
import warnings


class FrenetixHandler:
    def __init__(self, dt, ref_path, agent_params, initial_state, horizon=3.0):
        self.dt = dt
        self.agent_params = agent_params
        self.reference_path = ref_path
        self.trajectories = []
        self.initial_state = initial_state
        self.is_valid = True

        # try to create Curvilinear coordinate system
        try:
            self.cl_coordinate_system = CurvilinearCoordinateSystem(ref_path)
        except:
            warnings.warn('Could not create Curvilinear coordinate system for agent reference path')
            self.is_valid = False
            return

        delta_max = self.agent_params["delta_max"]
        wheelbase = self.agent_params["wheelbase"]
        switching_velocity = self.agent_params["switching_velocity"]
        max_acceleration = self.agent_params["max_acceleration"]
        velocity_delta_max = self.agent_params["velocity_delta_max"]

        self.trajectory_handler: frenetix.TrajectoryHandler = frenetix.TrajectoryHandler(dt=self.dt)
        self.sampling_handler = SamplingHandler(dt=self.dt, max_sampling_number=1, t_min=2.0, horizon=horizon,
                                                delta_d_max=0.5, delta_d_min=-0.5)
        self.coordinate_system_cpp: frenetix.CoordinateSystemWrapper = frenetix.CoordinateSystemWrapper(ref_path)

        self.trajectory_handler.add_feasability_function(ff.CheckYawRateConstraint(deltaMax=delta_max,
                                                                                   wheelbase=wheelbase,
                                                                                   wholeTrajectory=False))

        self.trajectory_handler.add_feasability_function(ff.CheckAccelerationConstraint(switchingVelocity=switching_velocity,
                                                                                        maxAcceleration=max_acceleration,
                                                                                        wholeTrajectory=False))

        self.trajectory_handler.add_feasability_function(ff.CheckCurvatureConstraint(deltaMax=delta_max,
                                                                                     wheelbase=wheelbase,
                                                                                     wholeTrajectory=False))

        self.trajectory_handler.add_feasability_function(ff.CheckCurvatureRateConstraint(wheelbase=wheelbase,
                                                                                         velocityDeltaMax=velocity_delta_max,
                                                                                         wholeTrajectory=False))

    def create_trajectories(self, horizon=3.0):

        # clear existing values
        self.trajectories.clear()

        # load orientation, velocity and position from initial state
        x0_orientation = self.initial_state.orientation
        initial_position = self.initial_state.position
        initial_velocity = self.initial_state.velocity

        # convert cartesian coordinates to curvilinear coordinates for trajectory generation
        s, d = self.cl_coordinate_system.convert_to_curvilinear_coords(initial_position[0], initial_position[1])

        # create sampling range
        samp_level = 0
        # t1_range = np.array(list(self.sampling_handler.t_sampling.to_range(samp_level)))
        t1_range = 3.0
        d1_range = np.array(list(self.sampling_handler.d_sampling.to_range(samp_level)))
        ss1_range = np.array([initial_velocity * 0.8, initial_velocity, initial_velocity * 1.2])

        # determine whether to use the low velocity mode or not
        if initial_velocity < 0.5:
            low_vel_mode = True
        else:
            low_vel_mode = False

        # create the sampling matrix using the given parameters
        sampling_matrix = generate_sampling_matrix(t0_range=0.0,
                                                   t1_range=t1_range,
                                                   s0_range=s,
                                                   ss0_range=initial_velocity,
                                                   sss0_range=0,
                                                   ss1_range=ss1_range,
                                                   sss1_range=0,
                                                   d0_range=d,
                                                   dd0_range=0,
                                                   ddd0_range=0,
                                                   d1_range=d1_range,
                                                   dd1_range=0.0,
                                                   ddd1_range=0.0)

        # add the "FillCoordinates" function to the handler --> needed to calculate the trajectory
        self.trajectory_handler.add_function(frenetix.trajectory_functions.FillCoordinates(
                                             lowVelocityMode=low_vel_mode,
                                             initialOrientation=x0_orientation,
                                             coordinateSystem=self.coordinate_system_cpp,
                                             horizon=int(horizon)))

        # reset all trajectories
        self.trajectory_handler.reset_Trajectories()

        # generate trajectories using the sampling matrix
        self.trajectory_handler.generate_trajectories(sampling_matrix, low_vel_mode)

        # evaluate the selected functions
        self.trajectory_handler.evaluate_all_current_functions(True)

        # store all trajectories in the list object
        for trajectory in self.trajectory_handler.get_sorted_trajectories():
            self.trajectories.append(trajectory)

# eof
