"""
mavsim_python: path viewer (for chapter 10)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import sys
sys.path.append("..")
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from viewers.drawing import drawMav, drawPath


class pathViewer():
    def __init__(self):
        self.scale = 4000
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('World Viewer')
        self.window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=self.scale, elevation=50, azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False  # has the mav been plotted yet?
        self.mav_plot = []
        self.path_plot = []

    def update(self, state, path):

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[1., 0., 0., 1]])
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.mav_plot = drawMav(state, self.window)
            self.path_plot = drawPath(path, red, self.window)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.mav_plot.update(state)
            if path.flag_path_changed:
                self.path_plot.update(path)
        # redraw
        self.app.processEvents()
