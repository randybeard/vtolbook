"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
    - Update history:
        5/8/2019 - R.W. Beard
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
from tools.transfer_function import transferFunction


class windSimulation:
    def __init__(self):
        Ts = SIM.ts_simulation
        # steady state wind defined in the inertial frame
        #self._steady_state = np.array([[0., 0., 0.]]).T
        self._steady_state = np.array([[0., 5., 0.]]).T

        #   Dryden gust model parameters (pg 56 UAV book)
        Va = 5 # must set Va to a constant value
        #
        Lu = 200.0
        Lv = 200.0
        Lw = 50.0
        gust_flag = False
        if gust_flag==True:
            sigma_u = 1.06
            sigma_v = 1.06
            sigma_w = 0.7
        else:
            sigma_u = 0.0
            sigma_v = 0.0
            sigma_w = 0.0
        a1 = sigma_u*np.sqrt(2.*Va/Lu)
        a2 = sigma_v*np.sqrt(3.*Va/Lv)
        a3 = a2*Va/np.sqrt(3)/Lv
        a4 = sigma_w*np.sqrt(3.*Va/Lw)
        a5 = a4*Va/np.sqrt(3)/Lw
        b1 = Va/Lu
        b2 = Va/Lv
        b3 = Va/Lw
        self.u_w = transferFunction(num=np.array([[a1]]),
                                     den=np.array([[1, b1]]),
                                     Ts=Ts)
        self.v_w = transferFunction(num=np.array([[a2, a3]]),
                                     den=np.array([[1, 2*b2, b2**2.0]]),
                                     Ts=Ts)
        self.w_w = transferFunction(num=np.array([[a4, a5]]),
                                     den=np.array([[1, 2*b3, b3**2.0]]),
                                     Ts=Ts)

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        #gust = np.array([[0.],[0.],[0.]])
        return np.concatenate(( self._steady_state, gust ))
