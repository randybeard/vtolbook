"""
quadrotorsim
       3/2/2022 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.quadrotor_parameters as QUAD


from chap2.quadrotor_viewer import QuadrotorViewer
from chap3.data_viewer import DataViewer
from chap4.quadrotor_dynamics import QuadrotorDynamics
from message_types.msg_autopilot import MsgAutopilot
from chap6.autopilot import Autopilot

# initialize the visualization
quadrotor_view = QuadrotorViewer()
data_view = DataViewer()

# initialize elements of the architecture
quadrotor = QuadrotorDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
trajectory = MsgAutopilot()

# initialize the simulation time
sim_time = SIM.start_time

# state machine resetting next waypoint
trajectory_state = 0

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------trajectory/waypoint generator-------------
    if trajectory_state == 0:
        if sim_time % 15 < SIM.ts_simulation:
            trajectory_state = 1
    elif trajectory_state == 1:
        # randomly select next waypoint
        trajectory.pos = np.array([
            [np.random.uniform(low=-10, high=10)],
            [np.random.uniform(low=-10, high=10)],
            [np.random.uniform(low=-10, high=-1)]])
        trajectory_state = 0
        print(trajectory.pos)

    # -------autopilot-------------
    estimated_state = quadrotor.true_state
    delta, commanded_state = autopilot.update(trajectory, estimated_state)

    # -------physical system-------------
    quadrotor.update(delta)

    # -------update viewer-------------
    quadrotor_view.update(quadrotor.true_state)
    data_view.update(quadrotor.true_state,  # true states
                     quadrotor.true_state,  # estimated states
                     commanded_state,  # commanded states
                     delta,  # inputs to the quadrotor
                     SIM.ts_simulation)

    # -------increment time-------------
    sim_time += SIM.ts_simulation





