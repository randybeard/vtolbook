"""
quadrotorsim
        3/18/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.quadrotor_parameters as QUAD


from chap2.quadrotor_viewer import QuadrotorViewer
from chap3.data_viewer import DataViewer
from chap3.quadrotor_dynamics import QuadrotorDynamics
from message_types.msg_delta import MsgDelta

# initialize the visualization
quadrotor_view = QuadrotorViewer()
data_view = DataViewer()

# initialize elements of the architecture
quadrotor = QuadrotorDynamics(SIM.ts_simulation)
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------vary forces and moments to check dynamics-------------
    delta.force = QUAD.mass * QUAD.gravity + 1.0
    delta.torque = np.array([[0.], [0.], [1.]])

    # -------physical system-------------
    quadrotor.update(delta)

    # -------update viewer-------------
    quadrotor_view.update(quadrotor.true_state)
    data_view.update(quadrotor.true_state,  # true states
                     quadrotor.true_state,  # estimated states
                     quadrotor.true_state,  # commanded states
                     delta,  # inputs to the quadrotor
                     SIM.ts_simulation)

    # -------increment time-------------
    sim_time += SIM.ts_simulation





