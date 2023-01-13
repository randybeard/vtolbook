"""
quadrotorsim
       3/2/2022 - RWB
       3/7/2022 - RWB
"""
import sys

sys.path.append('..')
sys.path.append('../viewers')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.convergence_parameters as VTOL

from viewers.vtol_viewer import vtolViewer
from viewers.state_viewer import stateViewer
from dynamics.vtol_dynamics import vtolDynamics
from viewers.thrust_torque_viewer import ThrustTorqueViewer

from quadrotor_controller.autopilot import Autopilot
from control_allocation.nonlinear_control_allocation import NonlinearControlAllocation
from rapid_landing_quadrotor.landing_trajectory import TrajectoryPlanner
from tools.msg_convert import msg_state2np
from tools.rotations import Quaternion2Rotation
import time

# initialize the visualization
vtol_view = vtolViewer()
DATA_VIEW = True
if DATA_VIEW:
    data_view = stateViewer()
    thrust_torque_view = ThrustTorqueViewer()

# initialize elements of the architecture
vtol = vtolDynamics()
autopilot = Autopilot(SIM.ts_simulation)
planner = TrajectoryPlanner(SIM.ts_simulation)
control_alloc = NonlinearControlAllocation()

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
        # 11, 17
        # 12, 18
        # 13, 20
        # 14, 20
        # 15, 22.6
        V0 = 15
        s = np.array([[-np.cos(gamma), 0, -np.sin(gamma)]]).T
        A = -VTOL.mass * VTOL.gravity + np.sqrt(VTOL.Tmax**2 - VTOL.mass**2 * VTOL.gravity**2 * np.cos(gamma)**2)
        tf = V0/A
        # vtol._state[6:10] = np.array([[0, 0, 0, 1]]).T # fipped around
        vtol._state[0:3] = V0**2 / 2 / A * s  # initial position
        vtol._state[3:6] = Quaternion2Rotation(vtol._state[6:10]) @ (-V0 * s)  # initial velocity
        vtol._update_true_state()
        # renew the trajectory from current position to final position
        trajectory= planner.renew(p0=vtol.true_state.position,
                                  v0=Quaternion2Rotation(vtol._state[6:10]).T @ vtol.true_state.velocity,
                                  pf=np.array([[0.], [0.], [0.]]),
                                  vf=np.array([[0.], [0.], [0.1]]),
                                  start_time=sim_time,
                                  end_time=sim_time + tf)
        planner_state = 1
    else:
        # get commanded pos, vel, accel, heading
        trajectory = planner.update(sim_time)
    # print(trajectory.pos)

    # -------autopilot-------------
    estimated_state = vtol.true_state
    force_torque, commanded_state = autopilot.update(trajectory, estimated_state)
    # thrust_torque = \
    #     np.concatenate((np.array([[0, force_torque.force]]).T, force_torque.torque), axis=0)
    delta = control_alloc.update(np.array([[0, -force_torque.force]]).T, 
        force_torque.torque, msg_state2np(estimated_state), vtol._Va)
    # print(-force_torque.force)
    # print(force_torque.torque)


    # -------physical system-------------
    vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics
    thrust_torque = np.concatenate([np.copy(vtol.total_thrust[[0,2]]), np.copy(vtol.total_torque)])
    thrust_torque_d = np.concatenate([np.array([[0, -force_torque.force]]).T, 
        force_torque.torque]).reshape((-1,1))
    # print(thrust_torque_d)
    # print(thrust_torque)

    # -------update viewer-------------
    vtol_view.update(vtol.true_state)
    if DATA_VIEW:
        data_view.update(msg_state2np(vtol.true_state),  # true states
                         msg_state2np(vtol.true_state),  # estimated states
                         msg_state2np(commanded_state),  # commanded states
                         delta,  # inputs to the quadrotor
                         SIM.ts_simulation)
        thrust_torque_view.update(thrust_torque.reshape(-1,1), thrust_torque_d.reshape(-1,1), SIM.ts_simulation)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

    # time.sleep(.5)



