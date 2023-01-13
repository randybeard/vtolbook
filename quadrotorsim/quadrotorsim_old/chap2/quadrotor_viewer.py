"""
vtol_viewer: simple vtol viewer
        4/1/2019 - R.W. Beard
        4/15/2019 - BGM
        5/3/2019 - R.W. Beard
        5/30/2019 - R.W. Beard
"""
import sys
sys.path.append("..")
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
from tools.drawing import DrawQuadrotor

class QuadrotorViewer():
    def __init__(self):
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Quadrotor Viewer')
        self.window.setGeometry(0, 0, 1000, 1000)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=200) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        self.vtol_plot = []

    def update(self, state):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.quadrotor_plot = DrawQuadrotor(state, self.window)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.quadrotor_plot.update(state)
        # update the center of the camera view to the quadrotor location
        # defined in ENU coordinates
        view_location = Vector(state.pos.item(1), state.pos.item(0), -state.pos.item(2))
        self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()
