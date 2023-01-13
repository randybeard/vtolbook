# VTOLsim

VTOLsim is a modified version of the python simulator that students develop in the Flight Dynamics class, based on the textbook _Small Unmanned Aircraft_ by Beard and McLain.
VTOLsim provides a dynamics implementation for the Convergence tilt-rotor winged eVTOL from EFlight.
Much of the theory for different parts of VTOLsim is described in the following resources.

## Installation

To set up the Python environment for running vtolsim, it can be helpful to create a Python virtual environment. This can be done by running the following commands:

```
python -m venv ~/.virtualenvs/vtolsim
source ~/.virtualenvs/vtolsim/bin/activate
```

Once within your Python environment, install necessary Python libraries. To do this, cd into vtolsim's main directory and run:

`cd geometric_control/ && pip3 install -r requirements.txt`

## Resources

* [State-Dependent LQR Control for a Tilt-Rotor UAV](https://ieeexplore.ieee.org/abstract/document/9147931)
  * Provides an initial reference on the dynamic model we use.
  * LQR controller in this paper is found in the `vtolsim/full_state_lqr` directory.
    * This controller performs very poorly and is not recommended for use.
  * Repository was at the `acc_2020_fig8`, `acc_2020_spline`, and `acc_2020_spline_theta0` git tags when submitted.
* [Nonlinear Trajectory Tracking Control for Winged eVTOL UAVs](https://scholarsarchive.byu.edu/facpub/5011/)
  * Submitted to 2021 ACC
  * Controller implementation found in the `vtolsim/geometric_control` directory.
  * Repository was at the `acc_2021` git tag when submitted.
* [Pitch and Thrust Allocation for Full-Flight-Regime Control of Winged eVTOL UAVs](https://scholarsarchive.byu.edu/facpub/5212/)
  * Submitted to L-CSS and 2021 CDC.
  * Improvements to the pitch and thrust allocation portion of the ACC 2021 paper.
  * Main implementation and plotting in `vtolsim/geometric_control/optimal_pitch.py` and `vtolsim/geometric_control/optimal_pitch_plots.py`
* [Trajectory Generation and Tracking Control for Winged Electric Vertical Takeoff and Landing Aircraft](https://scholarsarchive.byu.edu/etd/8952/)
  * Thesis. Describes the dynamics model, trajectory generation, and trajectory tracking controller.

## Trajectory Tracking Controller
The main value of this repository is in its implementation of the trajectory tracking controller.
This is found in `vtolsim/geometric_control`.
To run the controller, run `python3 geometric_control_sim.py`.
This tracks a predetermined trajectory.
To change the trajectory, you can use comments to choose the trajectory towards the top of `geometric_control_sim.py`.
The spline trajectories are created in `setup_polynomial_trajectory.py`.

Data from every simulation run is saved to `sim_data` as a `.npz` file.
This is then used by the various plotting scripts to create plots for papers.
`*.npz` files are included in the `.gitignore` to prevent them from cluttering the repository.

The `geometric_control_monte_carlo_paths_sim.py` simulation runs the geometric controller on many randomly generated BSpline trajectories. These are generated in `random_path.py`.

### Plotting
There are several different plotting scripts within the `geometric_control` directory.
The plotters are mostly modified by commenting/uncommenting lines in the repository.
For example, to produce `.pdf` files instead of `.png` files, the line `filetype="pdf"` should be uncommented.
At the bottom of the `acc2021_plotter.py` and `thesis_plotter.py` files are function calls for each of the different uses of the plotters (essentially for each of the different publication venues the plotter was used to generate plots for). This allows us to produce similar plots with different datasets to display different behaviors.
There is also a `thesis_fmt` flag in all of these files. To produce plots for papers, set `thesis_fmt=False`.

* `optimal_pitch_plots.py` creates the following plots used in Jacob Willis's thesis and in the CDC/L-CSS paper:
  * `CL_over_CD.pdf` - the L/D efficiency plot
  * `CLCD_comparison` - the CL and CD plots comparing the different lift and drag models.
  * `constant_F_gamma_increasing_V` - shows how the optimization responds to changing flight velocities.
  * `constant_F_V_changing_gamma` - shows how the optimization responds to changing flight path angles.
  * `constant_V_gamma_changing_F` - shows how the optimization responds to changing force angles.
  * **To Run:**
  * `python3 optimal_pitch_plots.py`
* `sim_data/acc2021_plotter.py` is used to generate the plots in the ACC 2021 paper, and the similar plots (showing the different pitch/thrust allocation methods) in the CDC/L-CSS paper.
  * The plots it produces are dependent on the uncommented lines at the bottom of the file. Each set of commented lines is labeled with the paper that they were used for.
  * to produce a set of plots, uncomment the desired set and run the file
    * `python3 acc2021_plotter.py`
* `sim_data/thesis_plotter.py` is very similar to `acc2021_plotter.py` but was used for producing figures 5.1 and 5.2 in Jacob Willis's thesis.
  * The plots it produces are dependent on the uncommented lines at the bottom of the file. Each set of commented lines is labeled with the paper that they were used for.
  * to produce a set of plots, uncomment the desired set and run the file
    * `python3 thesis_plotter.py`
* `sim_data/monte_carlo_plotter.py` plots the results of the `monte_carlo_paths_geometric_control_sim.py` simulation
  * to produce a set of plots, run the file
    * `python3 monte_carlo_plotter.py`


## AirSim Interface
We set up an interface with the "teleport" mode in AirSim. This interface is found in `viewers/airsim_demo.py`. The AirSim interface can be enabled by running AirSim with the `viewers/airsim_vtolsim_settings.json` file and by setting the `use_airsim` parameter in `parameters/simulation_parameters.py` to `True`.
Future work is to use the VTOL dynamics now implemented in the MAGICC lab's AirSim.
