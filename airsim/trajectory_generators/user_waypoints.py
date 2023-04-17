import cv2
import sys
sys.path.append(".")
from path_gen.pixel_to_ned import px_to_ned
import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import time

from hw2_dynamics_sim.drone import Drone, DroneState
from hw3_controller.trace_controller import SO3_Controller, MsgTrajectory
from hw1_dummy_control.visualize import AirSimViz


class UserDefinedPath:
    def __init__(self, num_pts=15, altitude=40) -> None:
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',self.on_mouse)
        self.waypoints = []
        self.pixel_waypoints = []
        self.alt = altitude

        self.img = cv2.imread("./path_gen/altitude_map.png")
        
        while len(self.waypoints) < num_pts:
            cv2.imshow('image',self.img)
            k = cv2.waitKey(10)
            if k == ord('x'):
                break
        cv2.destroyAllWindows()
        self.traj = BSplineTrajectory()
        res = self.traj.find_path_through_waypoints(np.array(self.waypoints)) # Where Brad 
        self.traj.check_feasible()

        seg = [5, 10, 20, 30, 40]
        ev = []
        it = []
        # fig, ax = plt.subplots(1,1)
        # fig2, ax2 = plt.subplots(1,1)
        # for s in np.arange(6,50,2):
        #     print("SEGMENTS", s)
        #     self.traj = BSplineTrajectory(num_segments=s)
        #     res = self.traj.find_path_through_waypoints(np.array(self.waypoints))
        #     if res.success:
        #         ax.scatter(s, res.nit, c='b')
        #         ax2.scatter(s, res.nfev, c='b')
        #     else:
        #         ax.scatter(s, 0, c='r')
        #         ax2.scatter(s, 0, c='r')

        # ax2.scatter(0, 0, c='r', label="fail")
        # ax.scatter(0, 0, c='r', label="fail")
        

        # ax.set_xlabel("Number of segments")
        # ax2.set_xlabel("Number of segments")
        # ax.set_ylabel("iterations")
        # ax2.set_ylabel("fun evalutations")
        # fig.legend()
        # fig2.legend()
        # plt.show()
                
        self.best_path = self.traj.spl
        # self.traj = BSplineTrajectory(num_segments=10)        
        # self.traj.find_path_through_waypoints(np.array(self.pixel_waypoints))        

            

    def on_mouse(self, event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.img = cv2.circle(self.img, (x,y), 5, (255,0,0), 2)
            north, east = px_to_ned(x,y)
            self.waypoints.append([north,east,self.alt])
            self.pixel_waypoints.append([x,y,0])


class BSplineTrajectory:
    def __init__(self, tf=1, num_segments=40, order=3) -> None:
        self.knots = self.uniform_knots(tf, order, num_segments)
        self.N = num_segments+order
        self.order=order
        self.g = np.array([[0,0,9.81]]).T
        self.tf = tf
        self.num_segments = num_segments


    def uniform_knots(self, tf, order, num_segments):
        t0=0
        knots = np.concatenate((
            t0 * np.ones(order),
            np.concatenate((
                np.arange(t0, tf, (tf-t0)/(num_segments)),
                tf * np.ones(order+1)),
                axis=0)
            ), axis=0)
        return knots

    def get_waypoint_times(self):
        distances = []
        total = 0
        for i, w in enumerate(self.waypoints):
            if i==0:
                continue

            diff = np.linalg.norm(w - self.waypoints[i-1])
            total +=diff
            distances.append(total)

        return np.array(distances)/total * self.tf
    

    def setup_cps(self):
        # Start and end at the first and last wp
        self.start_pt = self.waypoints[0,:].reshape(1,-1)
        self.end_pt = self.waypoints[-1,:].reshape(1,-1)

        # Clamp accel and vel to start at zero
        self.start_pts = np.tile(self.waypoints[0,:].reshape(1,-1), (3,1))
        self.end_pts = np.tile(self.waypoints[-1,:].reshape(1,-1), (3,1))
        self.middle = self.waypoints[1:-1,:]

        interp = np.linspace(0,1,self.N-6)
        self.ctrl_pts0 = (self.start_pt + (self.end_pt-self.start_pt)*interp[:,None])[:,:2]
        

    def find_path_through_waypoints(self, waypoints):
        self.waypoints = waypoints

        # Get distances between waypoints to calculate times
        
        wp_times = self.get_waypoint_times()
        self.setup_cps()

        
        # self.plot_traj(ctrl_pts0)
        
        # Set the maximum number of iterations to 1000
        max_iterations = 1000

        # Set the options parameter
        options = {'maxiter': max_iterations}
        cons_ineq = [{'type':'ineq', 'fun':self.wp_constraint, 'args': (self.middle, wp_times[:-1])}]
        opt_ctrl_pts = minimize(self.objective, self.ctrl_pts0, method='SLSQP', options=options, constraints=cons_ineq)
        print(opt_ctrl_pts)

        # Put together optimal path
        middle_pts = opt_ctrl_pts.x.reshape((-1,2))
        altitude = np.ones((middle_pts.shape[0],1))*self.start_pt[0,2]
        middle_pts = np.concatenate((middle_pts,altitude),1)

        self.ctrl_pts = np.concatenate((self.start_pts, middle_pts, self.end_pts), 0)
        self.spl = BSpline(t=self.knots, c=self.ctrl_pts, k=self.order)
        self.plot_traj()
        # self.check_feasible()
        return opt_ctrl_pts

    def objective(self, ctrl_pts, minimize_derivative=2):
        ctrl_pts = ctrl_pts.reshape((-1,2))
        altitude = np.ones((ctrl_pts.shape[0],1))*self.start_pt[0,2]
        ctrl_pts = np.concatenate((ctrl_pts,altitude),1)

        all_pts = np.concatenate((self.start_pts, ctrl_pts, self.end_pts), 0)
        path = BSpline(t=self.knots, c=all_pts, k=self.order)
        derivative_objective = path.derivative(minimize_derivative)

        # Integrate snap across knots
        t = np.linspace(self.knots[0], self.knots[-1], self.N)
        accel = derivative_objective(t)


        return np.linalg.norm(accel)

    def wp_constraint(self, ctrl_pts, wp, wp_ts, tol=1e-0):
        ctrl_pts = ctrl_pts.reshape((-1,2))
        altitude = np.ones((ctrl_pts.shape[0],1))*self.start_pt[0,2]
        ctrl_pts = np.concatenate((ctrl_pts,altitude),1)

        all_pts = np.concatenate((self.start_pts, ctrl_pts, self.end_pts), 0)
        path = BSpline(t=self.knots, c=all_pts, k=self.order)

        cons = []
        for idx, pt in enumerate(wp):
            t = wp_ts[idx]
            dist = np.linalg.norm(path(t) - pt)
            cons.append(tol - dist)
        # print(cons)
        return np.array(cons)
    
    def plot_traj(self):
        

        t0 = self.spl.t[0]  # first knot is t0
        tf = self.spl.t[-1]  # last knot is tf
        # number of points in time vector so spacing is 0.01
        N = int(np.ceil((tf - t0)/0.01))
        t = np.linspace(t0, tf, N)  # time vector
        position = self.spl(t)
        # 3D trajectory plot
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.spl.c[:, 0], self.spl.c[:, 1], self.spl.c[:, 2],
                '-o', label='control points')
        ax.plot(position[:, 0], position[:, 1], position[:, 2],
                'b', label='spline')
        
        # ax.scatter(self.start_pt, label="Start")
        if self.middle is not None:
            ax.scatter(self.middle[:,0], self.middle[:,1], self.middle[:,2], color='r', label = "waypoints")
        ax.scatter(self.start_pt[0,0], self.start_pt[0,1], self.start_pt[0,2], color='m', label="Start")
        ax.scatter(self.end_pt[0,0], self.end_pt[0,1], self.end_pt[0,2], color='g', label="End")
        #ax.set_xlim3d([-10, 10])
        ax.legend()
        ax.set_xlabel('x', fontsize=16, rotation=0)
        # ax.axes.set_xlim3d(0,10)
        # ax.axes.set_ylim3d(0,10)
        # ax.axes.set_zlim3d(0,10)
        ax.set_ylabel('y', fontsize=16, rotation=0)
        ax.set_zlabel('z', fontsize=16, rotation=0)
        ax.set_title("Num Segments: {}".format(self.num_segments))
        plt.show()

    def check_feasible(self):
        dt = 0.1
        uav = Drone(dt)
        cont = SO3_Controller(dt, uav)

        a_up = 1.
        a_low = 0.
        alpha = 1.
        feasible = False

        tf = self.tf
        T_up = 10.5
        T_low = 10.
        T_max = 0.
        knots0 = self.knots
        hist = []
        while not feasible:

            times = np.arange(0, self.knots[-1], dt)
            self.spl = BSpline(t=self.knots, c=self.ctrl_pts, k=self.order)
            print(alpha, T_max)
            
            feasible = True
            T_max = 0
            hist.append(alpha)
            for t in times:
                # p = spl(t)
                # v = spl.derivative(1)(t)
                a = self.spl.derivative(2)(t)
                T = self.flat_output_control(a)
                T_max = max([T, T_max])
                # print(t, T)

            if T_max > T_up:
                a_low = a_up
                a_up*=2
                alpha = (a_up+a_low)/2
                self.knots = knots0*alpha
                feasible = False
                # break

            elif T_max < T_low:
                a_up = alpha
                # a_low /=2
                alpha = (a_up+a_low)/2
                self.knots=alpha*knots0
                feasible = False
                # break

        fig, ax = plt.subplots(1,1)
        ax.set_xlabel("iteration")
        ax.set_ylabel(r"1/ $\alpha$")
        ax.plot(hist)
        plt.show()

        

            


    def flat_output_control(self, accel):
        T = np.linalg.norm(- self.g.T + accel)
        return T
        
def traj_to_msg(traj, t):
    p = traj(t)
    v = traj.derivative(1)(t)
    a = traj.derivative(2)(t)
    msg = MsgTrajectory()
    msg.set(p, v, a)
    return msg

if __name__ == "__main__":
    path = UserDefinedPath()

    traj = path.best_path

    dt = 0.01
    uav = Drone(dt) # Replace
    controller = SO3_Controller(dt, uav) # Replace
    viz = AirSimViz() # Replace 
    
    x0 = traj.c[0,:]
    uav.state.vec[:3] = x0.reshape(-1,1)
    tf = traj.t[-1]

    for t in np.arange(0,tf,dt):
        # Extract current desired state from spline
        command = traj_to_msg(traj, t) # Convert to 
        T, tau = controller.compute_control(uav.state, command)

        uav.update(T, tau)

        viz.set_pose(uav.state.p, uav.state.quat)
        # time.sleep(dt)
        

    


