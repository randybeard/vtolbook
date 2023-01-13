
import numpy as np

from trajectory import Trajectory


class TakeoffCoastLandTrajectory:
    def __init__(
            self,
            t_takeoff,
            t_coast,
            t_land,
            max_altitude,
            max_horizontal_speed):
        """
        Create a trajectory which uses constant acceleration to
        climb, coast, and descend.
        """
        self.t_takeoff = t_takeoff
        self.t_coast = t_coast
        self.t_land = t_land

        self.max_altitude = max_altitude
        self.max_horizontal_speed = max_horizontal_speed

        # climbing phase
        self.t1 = t_takeoff/2
        self.t2 = t_takeoff
        # coasting phase
        self.t3 = t_takeoff + t_coast/2
        self.t4 = t_takeoff + t_coast
        # descending phase
        self.t5 = t_takeoff + t_coast + t_land/2
        self.t6 = t_takeoff + t_coast + t_land

        self.takeoff_x_distance = (max_horizontal_speed/t_takeoff)*.5*t_takeoff**2
        self.landing_x_distance = (max_horizontal_speed/t_land)*.5*t_land**2
        self.final_x_position = (self.takeoff_x_distance
                                 + max_horizontal_speed*t_coast
                                 + self.landing_x_distance)

        self.yaw = 0.

        self.Nr = 4

    def eval(self, t, k):
        full_flag = self.evalUpToKr(t, 4)
        return full_flag[:,k]

    def evalUpToKr(self, t, kr):
        """
        Evaluate the trajectory and its krth derivatives
        """
        t_takeoff = self.t_takeoff 
        t_coast = self.t_coast 
        t_land = self.t_land 

        max_altitude = self.max_altitude 
        max_horizontal_speed = self.max_horizontal_speed 

        takeoff_x_distance = self.takeoff_x_distance 
        final_x_position = self.final_x_position 
        
        p_d0 = np.zeros(3)
        p_d1 = np.zeros(3)
        p_d2 = np.zeros(3)
        p_d3 = np.zeros(3)

        if t < self.t1:
            # climbing/accelerating phase
            p_d0 = np.array([(max_horizontal_speed/t_takeoff)*.5*t**2, # px_d0_t1
                    0*t, # py_d0_t1
                    -(max_altitude/(t_takeoff/2)**2)*.5*t**2]) # pz_d0_t1
            p_d1 = np.array([(max_horizontal_speed/t_takeoff)*t, # px_d1_t1
                    0*t, # py_d1_t1
                    -(max_altitude/(t_takeoff/2)**2)*t]) # pz_d1_t1
            p_d2 = np.array([(max_horizontal_speed/t_takeoff)*t**0, # px_d1_t1
                    0*t, # py_d1_t1
                    -(max_altitude/(t_takeoff/2)**2)*t**0]) # pz_d1_t1
        elif t < self.t2:
            # climbing/accelerating phase
            p_d0 = np.array([(max_horizontal_speed/t_takeoff)*.5*t**2, # px_d0_t2
                    0*t, # py_d0_t2
                    (max_altitude/(t_takeoff/2)**2)*.5*(t-t_takeoff)**2-max_altitude]) # pz_d0_t2
            p_d1 = np.array([(max_horizontal_speed/t_takeoff)*t, # px_d1_t2
                    0*t, # py_d1_t2
                    (max_altitude/(t_takeoff/2)**2)*(t-t_takeoff)]) # pz_d1_t2
            p_d2 = np.array([(max_horizontal_speed/t_takeoff)*t**0, # px_d1_t2
                    0*t, # py_d1_t2
                    (max_altitude/(t_takeoff/2)**2)*(t-t_takeoff)**0]) # pz_d1_t2
        elif t < self.t3:
            # coasting phase
            p_d0 = np.array([max_horizontal_speed*t-takeoff_x_distance, # px_d0_t3
                    0*t, # py_d0_t3
                    0*t - max_altitude]) # pz_d0_t3
            p_d1 = np.array([max_horizontal_speed*t**0, # px_d1_t3
                    0*t, # py_d1_t3
                    0*t]) # pz_d1_t3
            p_d2 = np.array([0*t, # px_d1_t3
                    0*t, # py_d1_t3
                    0*t]) # pz_d1_t3
        elif t < self.t4:
            # coasting phase
            p_d0 = np.array([max_horizontal_speed*t-takeoff_x_distance, # px_d0_t4
                    0*t, # py_d0_t4
                    0*t - max_altitude]) # pz_d0_t4
            p_d1 = np.array([max_horizontal_speed*t**0, # px_d1_t4
                    0*t, # py_d1_t4
                    0*t]) # pz_d1_t4
            p_d2 = np.array([0*t, # px_d1_t4
                    0*t, # py_d1_t4
                    0*t]) # pz_d1_t4
        elif t < self.t5:
            # descending phase
            p_d0 = np.array([-(max_horizontal_speed/t_land)*.5*(t-t_takeoff-t_coast-t_land)**2 + final_x_position, # px_d0_t5
                    0*t, # py_d0_t5
                    (max_altitude/(t_land/2)**2)*.5*(t-t_takeoff-t_coast)**2-max_altitude]) # pz_d0_t5
            p_d1 = np.array([-(max_horizontal_speed/t_land)*(t-t_takeoff-t_coast-t_land), # px_d1_t5
                    0*t, # py_d1_t5
                    (max_altitude/(t_land/2)**2)*(t-t_takeoff-t_coast)]) # pz_d1_t5
            p_d2 = np.array([-(max_horizontal_speed/t_land)*(t**0), # px_d1_t5
                    0*t, # py_d1_t5
                    (max_altitude/(t_land/2)**2)*t**0]) # pz_d1_t5
        elif t < self.t6:
            # descending phase
            p_d0 = np.array([-(max_horizontal_speed/t_land)*.5*(t-t_takeoff-t_coast-t_land)**2 + final_x_position, # px_d0_t6
                    0*t, # py_d0_t6
                    -(max_altitude/(t_land/2)**2)*.5*(t-t_takeoff-t_coast-t_land)**2]) # pz_d0_t6
            p_d1 = np.array([-(max_horizontal_speed/t_land)*(t-t_takeoff-t_coast-t_land), # px_d1_t6
                    0*t, # py_d1_t6
                    -(max_altitude/(t_land/2)**2)*(t-t_takeoff-t_coast-t_land)]) # pz_d1_t6
            p_d2 = np.array([-(max_horizontal_speed/t_land)*(t**0), # px_d1_t6
                    0*t, # py_d1_t6
                    -(max_altitude/(t_land/2)**2)*(t**0)]) # pz_d1_t6
        else: # t >= t6
            t = self.t6
            p_d0 = np.array([-(max_horizontal_speed/t_land)*.5*(t-t_takeoff-t_coast-t_land)**2 + final_x_position, # px_d0_t6
                    0*t, # py_d0_t6
                    -(max_altitude/(t_land/2)**2)*.5*(t-t_takeoff-t_coast-t_land)**2]) # pz_d0_t6
            p_d1 = np.zeros(3)
            p_d2 = np.zeros(3)

        p_d_all = np.column_stack([p_d0, p_d1, p_d2, p_d3])
        yaw_vec = np.array([self.yaw, 0., 0., 0.])
        full_flag = np.concatenate([p_d_all, yaw_vec.reshape(1,-1)], axis=0)

        return full_flag[:,:kr]

    def plot(self):
        import matplotlib.pyplot as plt
        t_end = self.t6
        t_vec = np.arange(0, t_end, .01)

        p_d0 = np.zeros((3,len(t_vec)))
        p_d1 = np.zeros((3,len(t_vec)))
        p_d2 = np.zeros((3,len(t_vec)))

        for i in range(len(t_vec)):
            t = t_vec[i]
            full_flag = self.evalUpToKr(t, 4)
            p_d0[:,i] = full_flag[0:3,0]
            p_d1[:,i] = full_flag[0:3,1]
            p_d2[:,i] = full_flag[0:3,2]
        ###########################
        # PLOTS
        fig, ax = plt.subplots(3,1)
        fig.suptitle("Position")
        ax[0].plot(t_vec, p_d0[0,:])
        ax[0].set_xticks(np.arange(0,t_end + 10,10))
        ax[0].grid()

        ax[1].plot(t_vec, p_d0[1,:])
        ax[1].set_xticks(np.arange(0,t_end + 10,10))
        ax[1].grid()

        ax[2].plot(t_vec, p_d0[2,:])
        ax[2].set_xticks(np.arange(0,t_end + 10,10))
        ax[2].grid()

        fig, ax = plt.subplots(3,1)
        fig.suptitle("Velocity")
        ax[0].plot(t_vec, p_d1[0,:])
        ax[0].set_xticks(np.arange(0,t_end + 10,10))
        ax[0].grid()

        ax[1].plot(t_vec, p_d1[1,:])
        ax[1].set_xticks(np.arange(0,t_end + 10,10))
        ax[1].grid()

        ax[2].plot(t_vec, p_d1[2,:])
        ax[2].set_xticks(np.arange(0,t_end + 10,10))
        ax[2].grid()


        fig, ax = plt.subplots(3,1)
        fig.suptitle("Acceleration")
        ax[0].plot(t_vec, p_d2[0,:])
        ax[0].set_xticks(np.arange(0,t_end + 10,10))
        ax[0].grid()

        ax[1].plot(t_vec, p_d2[1,:])
        ax[1].set_xticks(np.arange(0,t_end + 10,10))
        ax[1].grid()

        ax[2].plot(t_vec, p_d2[2,:])
        ax[2].set_xticks(np.arange(0,t_end + 10,10))
        ax[2].grid()

        fig, ax = plt.subplots(1,1)
        fig.suptitle("Trajectory")
        ax.plot(p_d0[0,:], p_d0[2,:])
        ax.set_xlabel(r"$p_x$")
        ax.set_ylabel(r"$p_z$")

        plt.show()


if __name__ == "__main__":
    # tcl = TakeoffCoastLandTrajectory(30, 20, 50, 50, 20)
    tcl = TakeoffCoastLandTrajectory(
        10, # t_takeoff
        10, # t_coast
        15, # t_land
        10, # max_altitude
        5) # max_horizontal_velocity
    tcl.plot()