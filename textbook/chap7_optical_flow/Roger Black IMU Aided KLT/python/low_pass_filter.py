# BYU MAGICC Lab
# date: April 2020
# name: Roger Black
# description: takes unfiltered acceleration and gyro data and returns it after passing it through a low pass filter

import params as P


class LowPassFilter:
    '''
        This class puts the acceleration/gyro data through a low pass filter
    '''

    def __init__(self):
        self.prev_accel_estimate = [0, 0, 0]
        self.accel_estimate = [0, 0, 0]
        self.prev_gyro_estimate = [0, 0, 0]
        self.gyro_estimate = [0, 0, 0]
        self.accel_alpha = P.accel_alpha
        self.gyro_alpha = P.gyro_alpha

    def get_accel_estimate(self, accel):
        i = 0
        for i in range(3):
            self.accel_estimate[i] = self.accel_alpha * accel[i] + (1 - self.accel_alpha) * self.prev_accel_estimate[i]
            self.prev_accel_estimate[i] = self.accel_estimate[i]

        return self.accel_estimate

    def get_gyro_estimate(self, gyro):
        i = 0
        for i in range(3):
            self.gyro_estimate[i] = self.gyro_alpha * gyro[i] + (1 - self.gyro_alpha) * self.prev_gyro_estimate[i]
            self.prev_gyro_estimate[i] = self.gyro_estimate[i]

        return self.gyro_estimate


