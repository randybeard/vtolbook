#!/usr/bin/python3

import numpy as np
from numpy.core.function_base import linspace
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import optimal_control_allocation


NUM_STEPS = 200
STEP_SIZE = 1 / NUM_STEPS

def thrust_torque_test(delta, Va, verbose=False):

    thrust0, torque0 = optimal_control_allocation.rotor_thrust_torque(delta, Va)
    thrust1, torque1, thrust_der1, torque_der1 = \
        optimal_control_allocation.rotor_thrust_torque_der(delta, Va)

    assert thrust0 == thrust1
    assert torque0 == torque1

    if verbose:
        print(f'thrust: {thrust0}')
        print(f'torque: {torque0}')
        print(f'thrust_der: {thrust_der1}')
        print(f'torque_der: {torque_der1}')
    
    return thrust0, torque0, thrust_der1, torque_der1

def plot_thrust_torque(plot_thrust=True, plot_euler_method=False):
    delta = linspace(0, 1, NUM_STEPS)
    delta_der = linspace(0, 1, 5)
    Va = [2.0, 2, 2]
    thrust = list()
    torque = list()
    thrust_der = list()
    torque_der = list()

    for d in delta:
        thrust_d, torque_d, thrust_der_d, torque_der_d = \
            thrust_torque_test([d, 0, 0], Va)
        thrust.append(thrust_d[0])
        torque.append(torque_d[0])
        if plot_euler_method:
            thrust_der.append(thrust_der_d[0])
            torque_der.append(torque_der_d[0])
    
    if not plot_euler_method:
        for d in delta_der:
            thrust_d, torque_d, thrust_der_d, torque_der_d = \
                thrust_torque_test([d, 0, 0], Va)
            thrust_der.append(thrust_der_d[0])
            torque_der.append(torque_der_d[0])

            if plot_thrust:
                y_intercept = thrust_d[0] - thrust_der_d[0] * d
                x = linspace(0, 1, NUM_STEPS)
                y = thrust_der_d[0] * x + y_intercept
                plt.plot(x, y)
            else:
                y_intercept = torque_d[0] - torque_der_d[0] * d
                x = linspace(0, 1, NUM_STEPS)
                y = torque_der_d[0] * x + y_intercept
                plt.plot(x, y)
    else:
        if plot_thrust:
            y = [thrust[0]]
            for i in range(len(delta) - 1):
                y.append(y[i] + STEP_SIZE * thrust_der[i])
        else:
            y = [torque[0]]
            for i in range(len(delta) - 1):
                y.append(y[i] + STEP_SIZE * torque_der[i])
        plt.plot(delta, y)

    if plot_thrust:
        plt.plot(delta, thrust)
    else:
        plt.plot(delta, torque)
    plt.show()

def plot_rotor_model(Va_bounds, rotor_num):

    def f(delta, Va):
        thrusts, _, _, _ = optimal_control_allocation.rotor_thrust_torque_der([delta, delta, delta], [Va, Va, Va])
        return thrusts[rotor_num]


    deltas = linspace(0, 1, 50) #[linspace(0, 1, 50), linspace(0, 1, 50), linspace(0, 1, 50)]
    Vas = linspace(Va_bounds[0], Va_bounds[1], 50) #[linspace(Va_bounds[0], Va_bounds[1], 50), linspace(Va_bounds[0], Va_bounds[1], 50), linspace(Va_bounds[0], Va_bounds[1], 50)]

    deltas_grid, Vas_grid = np.meshgrid(deltas, Vas)
    thrusts = f(deltas_grid, Vas_grid)

    # print(thrusts)
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.contour3D(deltas_grid, Vas_grid, thrusts, 100, cmap='viridis')
    ax.set_xlabel('throttle')
    ax.set_ylabel('airspeed through rotor')
    ax.set_zlabel('thrust')
    plt.show()
    # fig

# plot_thrust_torque(plot_euler_method=True, plot_thrust=False)

# delta = [0.5, 0, 0]
# Va = [2, 0, 0]
# thrust_torque_test(delta, Va)

# print('\n\n')
# delta = [0.51, 0, 0]
# Va = [2, 0, 0]
# thrust_torque_test(delta, Va)

plot_rotor_model([0, 25], 0)