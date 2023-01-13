"""
quadrotorsim
       3/2/2022 - RWB
       3/7/2022 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.quadrotor_parameters as QUAD

from chap2.quadrotor_viewer import QuadrotorViewer
from chap3.data_viewer import DataViewer
from chap4.quadrotor_dynamics import QuadrotorDynamics
from chap6.autopilot import Autopilot
from rapid_landing.landing_trajectory import TrajectoryPlanner

# initialize the visualization
quadrotor_view = QuadrotorViewer()
DATA_VIEW = True
if DATA_VIEW:
    data_view = DataViewer()

# initialize elements of the architecture
quadrotor = QuadrotorDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
planner = TrajectoryPlanner(SIM.ts_simulation)

# initialize the simulation time
sim_time = SIM.start_time

# planner state for initializing trajectory
planner_state = 0

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------trajectory/waypoint generator-------------
    if planner_state == 0:
        # set initial conditions
        gamma = np.radians(30)
        V0 = 15
        s = np.array([[np.cos(gamma), 0, -np.sin(gamma)]]).T
        A = -QUAD.mass * QUAD.gravity + np.sqrt(QUAD.Tmax**2 - QUAD.mass**2 * QUAD.gravity**2 * np.cos(gamma)**2)
        tf = V0/A
        quadrotor._state[0:3] = V0**2 / 2 / A * s  # initial position
        quadrotor._state[3:6] = -V0 * s  # initial velocity
        quadrotor.update_true_state()
        # renew the trajectory from current position to final position
        trajectory= planner.renew(p0=quadrotor.true_state.pos,
                                  v0=quadrotor.true_state.vel,
                                  pf=np.array([[0.], [0.], [0.]]),
                                  vf=np.array([[0.], [0.], [0.1]]),
                                  start_time=sim_time,
                                  end_time=sim_time + tf)
        planner_state = 1
    else:
        # get commanded pos, vel, accel, heading
        trajectory = planner.update(sim_time)

    # -------autopilot-------------
    estimated_state = quadrotor.true_state
    delta, commanded_state = autopilot.update(trajectory, estimated_state)

    # -------physical system-------------
    quadrotor.update(delta)

    # -------update viewer-------------
    quadrotor_view.update(quadrotor.true_state)
    if DATA_VIEW:
        data_view.update(quadrotor.true_state,  # true states
                         quadrotor.true_state,  # estimated states
                         commanded_state,  # commanded states
                         delta,  # inputs to the quadrotor
                         SIM.ts_simulation)

    # -------increment time-------------
    sim_time += SIM.ts_simulation





