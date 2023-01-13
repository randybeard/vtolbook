"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msgState
from message_types.msg_sensors import msgSensors
from message_types.msg_delta import msgDelta

import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.rotations import Quaternion2Rotation, Quaternion2Euler, Euler2Rotation

class mavDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.pn0],  # (0)
                               [MAV.pe0],   # (1)
                               [MAV.pd0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0]])   # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.true_state = msgState()
        # initialize the sensors message
        self._sensors = msgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=msgDelta())


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[3:6]
        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = self._state.item(10) \
                              + np.random.normal(SENSOR.gyro_x_bias, SENSOR.gyro_sigma)
        self._sensors.gyro_y = self._state.item(11) \
                              + np.random.normal(SENSOR.gyro_y_bias, SENSOR.gyro_sigma)
        self._sensors.gyro_z = self._state.item(12) \
                              + np.random.normal(SENSOR.gyro_z_bias, SENSOR.gyro_sigma)
        # simulate accelerometers(units of g)
        self._sensors.accel_x = self._forces.item(0)/MAV.mass + MAV.gravity*np.sin(theta) \
                            + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_y = self._forces.item(1)/MAV.mass - MAV.gravity*np.cos(theta)*np.sin(phi) \
                            + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_z = self._forces.item(2)/MAV.mass - MAV.gravity*np.cos(theta)*np.cos(phi) \
                            + np.random.normal(0., SENSOR.accel_sigma)
        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        R_mag = Euler2Rotation(0.0, np.radians(-66), np.radians(12.5))
        # magnetic field in inertial frame: unit vector
        mag_inertial = R_mag.T @ np.array([[1.0], [0.0], [0.0]])
        R = Quaternion2Rotation(self._state[6:10]) # body to inertial
        # magnetic field in body frame: unit vector
        mag_body = R.T @ mag_inertial
        self._sensors.mag_x = mag_body.item(0) + np.random.normal(0., SENSOR.mag_sigma)
        self._sensors.mag_y = mag_body.item(1) + np.random.normal(0., SENSOR.mag_sigma)
        self._sensors.mag_z = mag_body.item(2) + np.random.normal(0., SENSOR.mag_sigma)
        # simulate pressure sensors
        self._sensors.abs_pressure = -MAV.rho * MAV.gravity * self._state.item(2) \
                            + np.random.normal(0., SENSOR.abs_pres_sigma)
        self._sensors.diff_pressure = 0.5 * MAV.rho * self._Va**2 \
                            + np.random.normal(0., SENSOR.diff_pres_sigma)
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_n \
                             + np.random.normal(0., SENSOR.gps_n_sigma)
            self._gps_eta_e = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_e \
                             + np.random.normal(0., SENSOR.gps_e_sigma)
            self._gps_eta_h = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_h \
                             + np.random.normal(0., SENSOR.gps_h_sigma)
            self._sensors.gps_n = self._state.item(0) + self._gps_eta_n
            self._sensors.gps_e = self._state.item(1) + self._gps_eta_e
            self._sensors.gps_h = -self._state.item(2) + self._gps_eta_h
            self._sensors.gps_Vg = np.linalg.norm(self._state[3:6]) \
                                           + np.random.normal(0., SENSOR.gps_Vg_sigma)
            self._sensors.gps_course = np.arctan2(pdot.item(1), pdot.item(0)) \
                                      + np.random.normal(0., SENSOR.gps_course_sigma)
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation
        return self._sensors

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # pn = state.item(0)
        # pe = state.item(1)
        # pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        p_dot = Quaternion2Rotation(state[6:10]) @ state[3:6]
        pn_dot = p_dot.item(0)
        pe_dot = p_dot.item(1)
        pd_dot = p_dot.item(2)

        # position dynamics
        u_dot = r*v - q*w + fx/MAV.mass
        v_dot = p*w - r*u + fy/MAV.mass
        w_dot = q*u - p*v + fz/MAV.mass

        # rotational kinematics
        e0_dot = 0.5 * (-p*e1 - q*e2 - r*e3)
        e1_dot = 0.5 * (p*e0 + r*e2 - q*e3)
        e2_dot = 0.5 * (q*e0 - r*e1 + p*e3)
        e3_dot = 0.5 * (r*e0 + q*e1 -p*e2)

        # rotatonal dynamics
        p_dot = MAV.gamma1*p*q - MAV.gamma2*q*r + MAV.gamma3*l + MAV.gamma4*n
        q_dot = MAV.gamma5*p*r - MAV.gamma6*(p**2-r**2) + m/MAV.Jy
        r_dot = MAV.gamma7*p*q - MAV.gamma1*q*r + MAV.gamma4*l + MAV.gamma8*n

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        # convert wind vector from world to body frame
        R = Quaternion2Rotation(self._state[6:10]) # rotation from body to world frame
        wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
        wind_body_frame += gust  # add the gust
        self._wind = R @ wind_body_frame  # wind in the world frame
        # velocity vector relative to the airmass
        v_air = self._state[3:6] - wind_body_frame
        ur = v_air.item(0)
        vr = v_air.item(1)
        wr = v_air.item(2)
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        if ur == 0:
            self._alpha = np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan(wr/ur)
        # compute sideslip angle
        tmp = np.sqrt(ur**2 + wr**2)
        if tmp == 0:
            self._beta = np.sign(vr)*np.pi/2.
        else:
            self._beta = np.arcsin(vr/tmp)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        # compute gravitaional forces
        R = Quaternion2Rotation(self._state[6:10]) # rotation from body to world frame
        f_g = R.T @ np.array([[0.], [0.], [MAV.mass * MAV.gravity]])
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)

        # intermediate variables
        qbar = 0.5 * MAV.rho * self._Va**2
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)
        p_nondim = p * MAV.b / (2 * self._Va)  # nondimensionalize p
        q_nondim = q * MAV.c / (2 * self._Va)  # nondimensionalize q
        r_nondim = r * MAV.b / (2 * self._Va)  # nondimensionalize r

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-MAV.M * (self._alpha - MAV.alpha0))
        tmp2 = np.exp(MAV.M * (self._alpha + MAV.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
        CL = (1 - sigma) * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = MAV.C_D_p + ((MAV.C_L_0 + MAV.C_L_alpha * self._alpha)**2)/(np.pi * MAV.e * MAV.AR)
        # compute Lift and Drag Forces
        F_lift = qbar * MAV.S_wing * (
                CL
                + MAV.C_L_q * q_nondim
                + MAV.C_L_delta_e * delta.elevator
        )
        F_drag = qbar * MAV.S_wing * (
                CD
                + MAV.C_D_q * q_nondim
                + MAV.C_D_delta_e * delta.elevator
        )
        # compute longitudinal forces in body frame
        fx = fx - ca * F_drag + sa * F_lift
        fz = fz - sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy += qbar * MAV.S_wing * (
                MAV.C_Y_0
                + MAV.C_Y_beta * self._beta
                + MAV.C_Y_p * p_nondim
                + MAV.C_Y_r * r_nondim
                + MAV.C_Y_delta_a * delta.aileron
                + MAV.C_Y_delta_r * delta.rudder
        )
        # compute logitudinal torque in body frame
        My = qbar * MAV.S_wing * MAV.c * (
                MAV.C_m_0
                + MAV.C_m_alpha * self._alpha
                + MAV.C_m_q * q_nondim
                + MAV.C_m_delta_e * delta.elevator
        )
        # compute lateral torques in body frame
        Mx = qbar * MAV.S_wing * MAV.b * (
                MAV.C_ell_0
                + MAV.C_ell_beta * self._beta
                + MAV.C_ell_p * p_nondim
                + MAV.C_ell_r * r_nondim
                + MAV.C_ell_delta_a * delta.aileron
                + MAV.C_ell_delta_r * delta.rudder
        )
        Mz = qbar * MAV.S_wing * MAV.b * (
                MAV.C_n_0 + MAV.C_n_beta * self._beta
                + MAV.C_n_p * p_nondim
                + MAV.C_n_r * r_nondim
                + MAV.C_n_delta_a * delta.aileron
                + MAV.C_n_delta_r * delta.rudder
        )

        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)
        fx += thrust_prop
        Mx += -torque_prop

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller
        # map delta_t throttle command(0 to 1) into motor input voltage
        v_in = MAV.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop, 4)
             / (2.*np.pi)) * Va + MAV.KQ**2/MAV.R_motor
        c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop, 3) \
            * Va**2 - (MAV.KQ / MAV.R_motor) * v_in + MAV.KQ * MAV.i0
        # Angular speed of propeller
        omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        # thrust and torque due to propeller
        thrust_prop = (MAV.rho * np.power(MAV.D_prop,4) * MAV.C_T0 / (4 * np.pi**2)) * omega_p**2\
                      + (MAV.rho * np.power(MAV.D_prop, 3) * MAV.C_T1 * self._Va / 2 / np.pi) * omega_p\
                      + (MAV.rho * MAV.D_prop**2 * MAV.C_T2 * self._Va**2)
        torque_prop = (MAV.rho * np.power(MAV.D_prop, 5) * MAV.C_Q0 / (4 * np.pi**2)) * omega_p**2\
                      + (MAV.rho * np.power(MAV.D_prop, 4) * MAV.C_Q1 * self._Va / (2 * np.pi)) * omega_p\
                      + (MAV.rho * np.power(MAV.D_prop, 3) * MAV.C_Q2 * self._Va**2)
        return thrust_prop, torque_prop

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.pn = self._state.item(0)
        self.true_state.pe = self._state.item(1)
        self.true_state.h = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias
