"""
dirty derivative
"""
import numpy as np
import matplotlib.pyplot as plt

class DirtyDerivative:
    def __init__(self, Ts, tau):
        self.Ts = Ts
        self.tau = tau
        self.a1 = (2.0 * tau - Ts) / (2.0 * tau + Ts)
        self.a2 = 2.0 / (2.0 * tau+ Ts)
        self.initialized = False

    def update(self, z):
        if self.initialized is False:
            self.z_dot = 0 * z
            self.z_delay_1 = z
            self.initialized = True
        else:
            self.z_dot = self.a1 * self.z_dot \
                         + self.a2 * (z - self.z_delay_1)
            self.z_delay_1 = z
        return self.z_dot


if __name__ == "__main__":
    # instantiate the system
    Ts = 0.01  # simulation step size
    tau = 0.05  # dirty derivative gain (differentiate below 1/tau rad/sec)
    differentiator= DirtyDerivative(Ts, tau)

    # main simulation loop
    sim_time = 0.0
    y = np.cos(sim_time)
    ydot = differentiator.update(y)  # record output for plotting
    time_history = [sim_time]  # record time for plotting
    y_history = [y]
    ydot_history = [ydot]
    while sim_time < 10.0:
        sim_time += Ts   # increment the simulation time
        y = np.cos(sim_time)
        ydot = differentiator.update(y)  # record output for plotting
        time_history.append(sim_time)
        y_history.append(y)
        ydot_history.append(ydot)
    # plot output vs time
    plt.plot(time_history, y_history)
    plt.plot(time_history, ydot_history)
    plt.show()


