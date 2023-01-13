"""
compute_trim
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/29/2018 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.rotations import Euler2Quaternion
from message_types.msg_controls import msgControls

def compute_trim(mav, Va, gamma, servo0=0., servo_range=np.pi/6):
    # define initial state and input
    e0 = Euler2Quaternion(0., gamma, 0.)
    state0 = np.array([[mav._state.item(0)],  # pn 0
                   [mav._state.item(1)],  # pe 1
                   [mav._state.item(2)],  # pd 2
                   [mav._Va + 1.],  # u 3 - +1 to handle Va = 0.
                   [0.],  # v 4
                   [0.],  # w 5
                   [e0.item(0)],  # e0 6
                   [e0.item(1)],  # e1 7
                   [e0.item(2)],  # e2 8
                   [e0.item(3)],  # e3 9
                   [0.],  # p 10
                   [0.],  # q 11
                   [0.],  # r 12
                   [servo0],  # right rotor 13
                   [servo0]   # left rotor 14
                   ])
    delta0 = np.array([[0.],  # elevon_right 15
                       [0.],  # elevon_left 16
                       [1.0],  # throttle_rear 17
                       [1.0],  # throttle_right 18
                       [1.0],  # throttle_left 19
                       [servo0], # servo_right 20
                       [servo0]  # servo_left 21
                       ])
    x0 = np.concatenate((state0, delta0), axis=0)
    # define equality constraints
    cons = ([{'type': 'eq',
             'fun': lambda x: np.array([
                                x[3]**2 + x[4]**2 + x[5]**2 - Va**2,  # magnitude of velocity vector is Va
                                x[4],  # v=0, force side velocity to be zero
                                x[6]**2 + x[7]**2 + x[8]**2 + x[9]**2 - 1.,  # force quaternion to be unit length
                                x[7], # e1=0  - forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[9], # e3=0
                                x[10], # p=0  - angular rates should all be zero
                                x[11], # q=0
                                x[12], # r=0
                                x[13]-x[20], # initial right rotor=right rotor command
                                x[14]-x[21],  # initial left rotor=initial left rotor command
                                # x[17],
                                # x[18]-x[19],
                                # x[20],
                                # x[21]
                                ]),
             'jac': lambda x: np.array([
                                [0., 0., 0., 2*x[3], 2*x[4], 2*x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 2*x[6], 2*x[7], 2*x[8], 2*x[9], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., -1., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., -1.],
                                # [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                # [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., -1., 0., -0.],
                                # [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
                                # [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.]
                                ])
             }, {'type': 'ineq', #>0
            'fun': lambda x: np.array([
                               x[17], #3 motor inputs between 0 and 1
                               x[18],
                               x[19],
                               -x[17]+1.0,
                               -x[18]+1.0,
                               -x[19]+1.0,
                               x[20]+15.0*np.pi/180,  #tilt servos between -15 and 120 degrees
                               -x[20]+120.0*np.pi/180,
                               x[21]+15.0*np.pi/180,
                               -x[21]+120.0*np.pi/180,
                               x[20]+ servo_range - servo0,  # keep servos within 15 degrees of servo0
                               -x[20]+ servo_range + servo0,
                               x[21]+ servo_range - servo0,  # keep servos within 15 degrees of servo0
                               -x[21]+ servo_range + servo0
                               ]),
            'jac': lambda x: np.array([
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0., 0., 0., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0., 0., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
                               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1.]
                               ])
             }])
    # solve the minimization problem to find the trim states and inputs

    res = minimize(trim_objective_fun, x0, method='SLSQP', args = (mav, Va, gamma),
                   constraints=cons, options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
    # extract trim state and input and return
    trim_state = np.array([res.x[0:15]]).T
    trim_input = np.array([res.x[15:22]]).T
    print('trim_state=', trim_state.T)
    print('trim_input=', trim_input.T)
    return trim_state, trim_input

# objective function to be minimized
def trim_objective_fun(x, mav, Va, gamma):
    state = np.array([x[0:15]]).T
    delta = x[15:22]
    commands = msgControls()
    commands.elevon_right   = delta[0]
    commands.elevon_left    = delta[1]
    commands.throttle_rear  = delta[2]
    commands.throttle_right = delta[3]
    commands.throttle_left  = delta[4]
    commands.servo_right    = delta[5]
    commands.servo_left     = delta[6]
    xdot = np.array([[Va, 0., -Va*np.sin(gamma), 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T
    mav._state = state
    mav._update_velocity_data()
    forces_moments = mav._forces_moments(commands)
    f = mav._derivatives(state, forces_moments, commands)
    tmp = xdot - f
    J = np.linalg.norm(tmp[0:15])**2.0
    return J
