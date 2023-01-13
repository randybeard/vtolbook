"""
holodeck_viewer:
        6/18/2020 - R.W. Beard
"""
import sys
sys.path.append("..")
import holodeck
import cv2
from tools.rotations import rotation_to_euler
import numpy as np


class holodeckWorld():
    def __init__(self):
        # load and initialize the holodeck world
        self.env = holodeck.make("UrbanCity-MaxDistance")
        #self.env = holodeck.make("CyberPunkCity-Follow")
        self.env.reset()

    def update(self, state):
        pos = state.pos.flatten()
        pos[2] = -pos[2]  # change NEU to NED
        phi, theta, psi = rotation_to_euler(state.rot)
        rot = (180 / np.pi) * np.array([phi, theta, psi])
        vel = state.vel.flatten()
        vel[2] = -vel[2]  # change NEU to NED
        omega = state.omega.flatten()
        self.env.set_state(agent_name='uav0',
                           location=pos,
                           rotation=rot,
                           velocity=vel,
                           angular_velocity=omega)
        # rotate the gimbal
        tmp = (180/np.pi) * state.gimbal
        gimbal_ang = [tmp.item(2), tmp.item(1), tmp.item(0)]
        self.env.agents['uav0'].sensors['RGBCamera'].rotate(gimbal_ang)
        # update and display the camera pixel array
        sensors = self.env.tick()
        pixels = sensors['RGBCamera']
        cv2.namedWindow("Camera Output")
        cv2.moveWindow("Camera Output", 500, 500)
        cv2.imshow("Camera Output", pixels[:, :, 0:3])
        cv2.waitKey(1)
        # cv2.destroyAllWindows()
        return pixels



