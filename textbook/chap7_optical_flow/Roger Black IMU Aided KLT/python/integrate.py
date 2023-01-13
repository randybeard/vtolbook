# BYU MAGICC Lab
# date: April 2020
# name: Roger Black
# description: Integrates acceleration and gyro data to get position data. Integration performed using the trapezoidal
# rule

import numpy as np
import params as P


class Integrate:
    '''
        This class uses the trapezoidal rule to integrate acceleration to get velocity and velocity to get position
    '''

    def __init__(self):
        self.prev_accel = [0, 0, 0]
        self.prev_vel = [0, 0, 0]
        self.prev_gyro = [0, 0, 0]
        self.dt = P.dt
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.ang_pos = [0, 0, 0]

    def gyro_to_position(self, gyro):
        i = 0
        for i in range(len(gyro)):
            # a gyro gives data in rates (velocity)
            # integrate velocity to get position
            self.ang_pos[i] += np.trapz(y=[self.prev_gyro[i], gyro[i]], dx=self.dt)

        self.prev_gyro = gyro

        return self.ang_pos

    def acceleration_to_position(self, accel):
        i = 0
        for i in range(len(accel)):
            # integrate acceleration to get velocity
            self.vel[i] += np.trapz(y=[self.prev_accel[i], accel[i]], dx=self.dt)
            # TODO write velocity history to storage and/or plot

            # integrate velocity to get position
            self.pos[i] += np.trapz(y=[self.prev_vel[i], self.vel[i]], dx=self.dt)

        self.prev_accel = accel
        self.prev_vel = self.vel

        return self.pos

    def reset(self):
        self.prev_accel = [0, 0, 0]
        self.prev_vel = [0, 0, 0]
        self.prev_gyro = [0, 0, 0]
        self.dt = P.dt
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.ang_pos = [0, 0, 0]


