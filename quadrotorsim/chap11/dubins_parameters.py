# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB

import numpy as np
import sys
sys.path.append('..')


class dubinsParameters:
    def __init__(self):
        self.p_s = np.inf*np.ones((3,1))  # the start position in re^3
        self.chi_s = np.inf  # the start course angle
        self.p_e = np.inf*np.ones((3,1))  # the end position in re^3
        self.chi_e = np.inf  # the end course angle
        self.radius = np.inf  # turn radius
        self.length = np.inf  # length of the Dubins path
        self.center_s = np.inf*np.ones((3,1))  # center of the start circle
        self.dir_s = np.inf  # direction of the start circle
        self.center_e = np.inf*np.ones((3,1))  # center of the end circle
        self.dir_e = np.inf  # direction of the end circle
        self.r1 = np.inf*np.ones((3,1))  # vector in re^3 defining half plane H1
        self.r2 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H2
        self.r3 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H3
        self.n1 = np.inf*np.ones((3,1))  # unit vector in re^3 along straight line path
        self.n3 = np.inf*np.ones((3,1))  # unit vector defining direction of half plane H3

    def update(self, ps, chis, pe, chie, R):
        ell = np.linalg.norm(ps[0:2] - pe[0:2])
        if ell < 2 * R:
            print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
        else:
            # compute start and end circles
            crs = ps + R * rotz(np.pi/2) @ np.array([[np.cos(chis)], [np.sin(chis)], [0]])
            cls = ps + R * rotz(-np.pi/2) @ np.array([[np.cos(chis)], [np.sin(chis)], [0]])
            cre = pe + R * rotz(np.pi/2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
            cle = pe + R * rotz(-np.pi/2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])

            # compute L1
            theta = np.arctan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0))
            L1 = np.linalg.norm(crs-cre)\
                 + R * mod(2*np.pi + mod(theta - np.pi/2) - mod(chis - np.pi/2) )\
                 + R * mod(2*np.pi + mod(chie - np.pi/2) - mod(theta - np.pi/2) )
            # compute L2
            ell = np.linalg.norm(cle - crs)
            theta = np.arctan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))
            theta2 = theta - np.pi/2 + np.arcsin(2 * R / ell)
            if not np.isreal(theta2):
                L2 = np.inf
            else:
                L2 = np.sqrt(ell**2 - 4 * R**2)\
                     + R * mod(2*np.pi + mod(theta2) - mod(chis - np.pi/2) )\
                     + R * mod(2*np.pi + mod(theta2 + np.pi) - mod(chie + np.pi/2) )

            # compute L3
            ell = np.linalg.norm(cre-cls)
            theta = np.arctan2(cre.item(1) - cls.item(1), cre.item(0) - cls.item(0))
            theta2 = np.arccos(2 * R / ell)
            if not np.isreal(theta2):
                L3 = np.inf
            else:
                L3 = np.sqrt(ell**2 - 4 * R**2)\
                     + R * mod(2*np.pi - mod(theta + theta2) + mod(chis + np.pi/2) )\
                     + R * mod(2*np.pi - mod(theta + theta2 - np.pi) + mod(chie - np.pi/2) )
            # compute L4
            theta = np.arctan2(cle.item(1) - cls.item(1), cle.item(0) - cls.item(0))
            L4 = np.linalg.norm(cls-cle)\
                 + R * mod(2*np.pi - mod(theta + np.pi/2) + mod(chis + np.pi/2) )\
                 + R * mod(2*np.pi - mod(chie + np.pi/2) + mod(theta + np.pi/2) )
            # L is the minimum distance
            L = np.min([L1, L2, L3, L4])
            idx = np.argmin([L1, L2, L3, L4])
            e1 = np.array([[1, 0, 0]]).T
            if idx == 0:
                cs = crs
                lams = 1
                ce = cre
                lame = 1
                q1 = (ce-cs)/np.linalg.norm(ce-cs)
                w1 = cs + R * rotz(-np.pi/2) @ q1
                w2 = ce + R * rotz(-np.pi/2) @ q1
            elif idx == 1:
                cs = crs
                lams = 1
                ce = cle
                lame = -1
                ell = np.linalg.norm(ce-cs)
                theta = np.arctan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                theta2 = theta - np.pi/2 + np.arcsin(2 * R / ell)
                q1 = rotz(theta2 + np.pi/2) @ e1
                w1 = cs + R *rotz(theta2) @ e1
                w2 = ce + R * rotz(theta2 + np.pi) @ e1
            elif idx == 2:
                cs = cls
                lams = -1
                ce = cre
                lame = 1
                ell = np.linalg.norm(ce-cs)
                theta = np.arctan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                theta2 = np.arccos(2 * R / ell)
                q1 = rotz(theta + theta2 - np.pi/2) @ e1
                w1 = cs + R * rotz(theta + theta2) @ e1
                w2 = ce + R * rotz(-np.pi + theta + theta2) @ e1
            elif idx == 3:
                cs = cls
                lams = -1
                ce = cle
                lame = -1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                w1 = cs + R * rotz(np.pi/2) @ q1
                w2 = ce + R * rotz(np.pi/2) @ q1
            w3 = pe
            q3 = rotz(chie) @ e1

            self.p_s = ps
            self.chi_s = chis
            self.p_e = pe
            self.chi_e = chie
            self.radius = R
            self.length = L
            self.center_s = cs
            self.dir_s = lams
            self.center_e = ce
            self.dir_e = lame
            self.r1 = w1
            self.n1 = q1
            self.r2 = w2
            self.r3 = w3
            self.n3 = q3


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


