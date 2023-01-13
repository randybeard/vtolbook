# Trajectory Generator
Generate multi-dimensional, differentiable, time-dependent trajectories. 
Two ROS nodes are provided, one for sinusoid trajectories and one for spline trajectories.
The generators can also be used independent of ROS.

## Sinusoid trajectories
Can be used to generate continuous orbits, figure-8, etc.
Good for testing controllers.
See [quadrotor_sinusoid_example.py](./scripts/quadrotor_sinusoid_example.py) for a walkthrough.

## Spline trajectories
Generates a smooth trajectory between an arbitrary number of points and satisfying an arbitrary order of derivative constraints at each point.
See [polynomial_example.py](./scripts/polynomial_example.py) for a walkthrough.
