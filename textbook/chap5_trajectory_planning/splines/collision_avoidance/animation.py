from matplotlib import pyplot as plt
from matplotlib import patches as mpatches
import numpy as np 
import param as P


class Animation:
    def __init__(self):
        self.flag_init = True  # Used to indicate initialization
        # Initialize a figure and axes object
        self.fig, self.ax = plt.subplots()
        # Initializes a list of objects (patches and lines)
        self.handle = []
        # Specify the x,y axis limits
        plt.axis([-P.size_E, P.size_E, -P.size_N, P.size_N])
        # label axes
        plt.xlabel('East')
        plt.ylabel('North')
        self.ax.axis('equal')
        # aircraft points
        self.pts = P.uav_size * np.array([
            [1, 1],
            [3, 1],
            [3, -1],
            [1, -1],
            [1, -5],
            [-1, -5],
            [-1, -1],
            [-4, -1],
            [-4, -3],
            [-5, -3],
            [-5, 3],
            [-4, 3],
            [-4, 1],
            [-1, 1],
            [-1, 5],
            [1, 5],
            [1, 1],
        ]).T

    def update(self, ownship, intruder):
        self.draw_ownship(ownship)
        self.draw_intruder(intruder)
        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False
        #plt.show()

    def draw_ownship(self, ownship):
        theta = np.arctan2(ownship.vel[1][0], ownship.vel[0][0])
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ])
        pts = R @ self.pts
        pts = pts + np.tile(ownship.pos[0:2], (1, pts.shape[1]))
        R_axis = np.array([[0, 1], [1, 0]])
        pts = R_axis @ pts
        xy = np.array(pts.T)

        # When the class is initialized, a polygon patch object will be
        # created and added to the axes. After initialization, the polygon
        # patch object will only be updated.
        if self.flag_init == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Polygon(xy, facecolor='blue', edgecolor='black'))
            self.ax.add_patch(self.handle[0]) # Add the patch to the axes
        else:
            self.handle[0].set_xy(xy)         # Update polygon

    def draw_intruder(self, intruder):
        theta = np.arctan2(intruder.vel[1][0], intruder.vel[0][0])
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ])
        pts = R @ self.pts
        pts = pts + np.tile(intruder.pos[0:2], (1, pts.shape[1]))
        R_axis = np.array([[0, 1], [1, 0]])
        pts = R_axis @ pts
        xy = np.array(pts.T)
        U,S,V = np.linalg.svd(intruder.covar)

        # When the class is initialized, a polygon patch object will be
        # created and added to the axes. After initialization, the polygon
        # patch object will only be updated.
        if self.flag_init == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Polygon(xy, facecolor='red', edgecolor='black'))
            self.ax.add_patch(self.handle[1]) # Add the patch to the axes
            self.handle.append(mpatches.Ellipse(xy=R_axis @ intruder.pos[0:2], 
                                                width=S[0], height=S[1], 
                                                angle = np.arccos(U[0][0]),
                                                facecolor='None', edgecolor='red'))
            self.ax.add_patch(self.handle[2]) # Add the patch to the axes
        else:
            self.handle[1].set_xy(xy)
            self.handle[2].set_center(R_axis @ intruder.pos[0:2])         # Update ellispe
            self.handle[2].set_width = S[0]
            self.handle[2].set_height = S[1]
            self.handle[2].set_angle = np.arccos(U[0][0])

