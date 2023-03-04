
import matplotlib.pyplot as plt
import param as P
from signalGenerator import signalGenerator
from animation import Animation
import numpy as np
from intruderDynamics import IntruderDynamics
from ownshipDynamics import OwnshipDynamics

#from dataPlotter import dataPlotter
#from ctrlStateFeedback import ctrlStateFeedback

# instantiate pendulum, controller, and reference classes
#pendulum = pendulumDynamics()
#controller = ctrlStateFeedback()
#reference = signalGenerator(amplitude=0.5, frequency=0.04)
#disturbance = signalGenerator(amplitude=0.1)

# instantiate the simulation plots and animation
#dataPlot = dataPlotter()
animation = Animation()
ownship = OwnshipDynamics()
intruder = IntruderDynamics()


t = P.t_start  # time starts at t_start
#y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot: 
        #r = reference.square(t)  # reference input
        #d = disturbance.step(t)  # input disturbance
        #n = 0.0  #noise.random(t)  # simulate sensor noise
        #x = pendulum.state
        #u = controller.update(r, x)  # update controller
        #y = pendulum.update(u + d)  # propagate system
        u = 0.0
        ownship.update(u)
        intruder.update()        
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(ownship, intruder)
    #dataPlot.update(t, r, pendulum.state, u)
    plt.pause(0.0001)  

# Keeps the program from closing until user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
