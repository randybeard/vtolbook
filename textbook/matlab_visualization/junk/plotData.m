classdef plotData < handle
    %----------------------------
    properties
        % data histories
        time_history
        % actual states
        n_history, e_history, d_history
        u_history, v_history, w_history
        phi_history, theta_history, psi_history
        p_history, q_history, r_history
        azimuth_history, elevation_history
        F_history
        tau_x_history, tau_y_history, tau_z_history
        % desired states
        n_des_history, e_des_history, d_des_history
        u_des_history, v_des_history, w_des_history
        phi_des_history, theta_des_history, psi_des_history
        p_des_history, q_des_history, r_des_history
        % estimated states
        n_hat_history, e_hat_history, d_hat_history
        u_hat_history, v_hat_history, w_hat_history
        phi_hat_history, theta_hat_history, psi_hat_history
        p_hat_history, q_hat_history, r_hat_history
        index
        % figure handles
        % actual states
        n_handle, e_handle, d_handle
        u_handle, v_handle, w_handle
        phi_handle, theta_handle, psi_handle
        p_handle, q_handle, r_handle
        azimuth_handle, elevation_handle
        F_handle
        tau_x_handle, tau_y_handle, tau_z_handle
        % desired states
        n_des_handle,  e_des_handle, d_des_handle
        u_des_handle, v_des_handle, w_des_handle
        phi_des_handle, theta_des_handle, psi_des_handle
        p_des_handle, q_des_handle, r_des_handle
        % estimated states
        n_hat_handle, e_hat_handle, d_hat_handle
        u_hat_handle, v_hat_handle, w_hat_handle
        phi_hat_handle, theta_hat_handle, psi_hat_handle
        p_hat_handle, q_hat_handle, r_hat_handle
    end
    methods
        %--constructor--------------------------
        function self = plotData(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            % actual states
            self.n_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.e_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.d_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.u_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.v_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.w_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.phi_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.psi_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.p_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.q_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.r_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.azimuth_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.elevation_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.F_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.tau_x_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.tau_y_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.tau_z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            % desired states
            self.n_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.e_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.d_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.u_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.v_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.w_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.phi_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.psi_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.p_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.q_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.r_des_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            % estimated states
            self.n_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.e_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.d_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.u_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.v_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.w_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.phi_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.psi_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.p_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.q_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.r_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            title('Quadrotor Data')
            subplot(6, 3, 1)
                hold on
                self.n_hat_handle    = plot(self.time_history, self.n_hat_history, 'g');
                self.n_des_handle    = plot(self.time_history, self.n_des_history, 'r');
                self.n_handle    = plot(self.time_history, self.n_history, 'b');
                ylabel('n')
            subplot(6, 3, 4)
                hold on
                self.e_hat_handle    = plot(self.time_history, self.e_hat_history, 'g');
                self.e_des_handle    = plot(self.time_history, self.e_des_history, 'r');
                self.e_handle    = plot(self.time_history, self.e_history, 'b');
                ylabel('e')
            subplot(6, 3, 7)
                hold on
                self.d_hat_handle    = plot(self.time_history, -self.d_hat_history, 'g');
                self.d_des_handle    = plot(self.time_history, -self.d_des_history, 'r');
                self.d_handle    = plot(self.time_history, -self.d_history, 'b');
                ylabel('h')
            subplot(6, 3, 10)
                hold on
                self.u_hat_handle    = plot(self.time_history, self.u_hat_history, 'g');
                self.u_des_handle    = plot(self.time_history, self.u_des_history, 'r');
                self.u_handle    = plot(self.time_history, self.u_history, 'b');
            subplot(6, 3, 13)
                hold on
                self.v_hat_handle    = plot(self.time_history, self.v_hat_history, 'g');
                self.v_des_handle    = plot(self.time_history, self.v_des_history, 'r');
                self.v_handle    = plot(self.time_history, self.v_history, 'b');
                ylabel('v')
            subplot(6, 3,16)
                hold on
                self.w_hat_handle    = plot(self.time_history, self.w_hat_history, 'g');
                self.w_des_handle    = plot(self.time_history, self.w_des_history, 'r');
                self.w_handle    = plot(self.time_history, self.w_history, 'b');
                ylabel('w')
            subplot(6, 3, 2)
                hold on
                self.phi_hat_handle    = plot(self.time_history, self.phi_hat_history, 'g');
                self.phi_des_handle    = plot(self.time_history, self.phi_des_history, 'r');
                self.phi_handle    = plot(self.time_history, self.phi_history, 'b');
                ylabel('phi')
            subplot(6, 3, 5)
                hold on
                self.theta_hat_handle    = plot(self.time_history, self.theta_hat_history, 'g');
                self.theta_des_handle    = plot(self.time_history, self.theta_des_history, 'r');
                self.theta_handle    = plot(self.time_history, self.theta_history, 'b');
                ylabel('theta')
            subplot(6, 3, 8)
                hold on
                self.psi_hat_handle    = plot(self.time_history, self.psi_hat_history, 'g');
                self.psi_des_handle    = plot(self.time_history, self.psi_des_history, 'r');
                self.psi_handle    = plot(self.time_history, self.psi_history, 'b');
                ylabel('psi')
            subplot(6, 3, 11)
                hold on
                self.p_hat_handle    = plot(self.time_history, self.p_hat_history, 'g');
                self.p_des_handle    = plot(self.time_history, self.p_des_history, 'r');
                self.p_handle    = plot(self.time_history, self.p_history, 'b');
                ylabel('p')
            subplot(6, 3, 14)
                hold on
                self.q_hat_handle    = plot(self.time_history, self.q_hat_history, 'g');
                self.q_des_handle    = plot(self.time_history, self.q_des_history, 'r');
                self.q_handle    = plot(self.time_history, self.q_history, 'b');
                ylabel('q')
            subplot(6, 3, 17)
                hold on
                self.r_hat_handle    = plot(self.time_history, self.r_hat_history, 'g');
                self.r_des_handle    = plot(self.time_history, self.r_des_history, 'r');
                self.r_handle    = plot(self.time_history, self.r_history, 'b');
                ylabel('r')
            subplot(6, 3, 3)
                hold on
                self.azimuth_handle    = plot(self.time_history, self.azimuth_history, 'b');
                ylabel('azimuth')
            subplot(6, 3, 6)
                hold on
                self.elevation_handle    = plot(self.time_history, self.elevation_history, 'b');
                ylabel('elevation')
            subplot(6, 3, 9)
                hold on
                self.F_handle    = plot(self.time_history, self.F_history, 'b');
                ylabel('F')
            subplot(6, 3, 12)
                hold on
                self.tau_x_handle    = plot(self.time_history, self.tau_x_history, 'b');
                ylabel('tau_x')
            subplot(6, 3, 15)
                hold on
                self.tau_y_handle    = plot(self.time_history, self.tau_y_history, 'b');
                ylabel('tau_y')
            subplot(6, 3, 18)
                hold on
                self.tau_z_handle    = plot(self.time_history, self.tau_z_history, 'b');
                ylabel('tau_z')
        end
        %----------------------------
        function self = updatePlots(self, t, states, ctrl, desired_states, estimated_states)
            % convert quaternion to Euler angles
            Theta = utilities.quat2euler(states(7:10));
            phi = Theta(1);
            theta = Theta(2);
            psi = Theta(3);
            Theta_des = utilities.quat2euler(desired_states(7:10));
            phi_des = Theta_des(1);
            theta_des = Theta_des(2);
            psi_des = Theta_des(3);
            Theta_hat = utilities.quat2euler(estimated_states(7:10));
            phi_hat = Theta_hat(1);
            theta_hat = Theta_hat(2);
            psi_hat = Theta_hat(3);

            % update the time history of all plot variables
            self.time_history(self.index) = t;
            % actual states
            self.n_history(self.index) = states(1);
            self.e_history(self.index) = states(2);
            self.d_history(self.index) = states(3);
            self.u_history(self.index) = states(4);
            self.v_history(self.index) = states(5);
            self.w_history(self.index) = states(6);
            self.phi_history(self.index) = 180/pi*phi;
            self.theta_history(self.index) = 180/pi*theta;
            self.psi_history(self.index) = 180/pi*psi;
            self.p_history(self.index) = 180/pi*states(11);
            self.q_history(self.index) = 180/pi*states(12);
            self.r_history(self.index) = 180/pi*states(13);
            self.azimuth_history(self.index) = 180/pi*states(14);
            self.elevation_history(self.index) = 180/pi*states(15);
            self.F_history(self.index) = ctrl(1);
            self.tau_x_history(self.index) = ctrl(2);
            self.tau_y_history(self.index) = ctrl(3);
            self.tau_z_history(self.index) = ctrl(4);
            % desired states
            self.n_des_history(self.index) = desired_states(1);
            self.e_des_history(self.index) = desired_states(2);
            self.d_des_history(self.index) = desired_states(3);
            self.u_des_history(self.index) = desired_states(4);
            self.v_des_history(self.index) = desired_states(5);
            self.w_des_history(self.index) = desired_states(6);
            self.phi_des_history(self.index) = 180/pi*phi_des;
            self.theta_des_history(self.index) = 180/pi*theta_des;
            self.psi_des_history(self.index) = 180/pi*psi_des;
            self.p_des_history(self.index) = 180/pi*desired_states(11);
            self.q_des_history(self.index) = 180/pi*desired_states(12);
            self.r_des_history(self.index) = 180/pi*desired_states(13);
            % estimated states
            self.n_hat_history(self.index) = estimated_states(1);
            self.e_hat_history(self.index) = estimated_states(2);
            self.d_hat_history(self.index) = estimated_states(3);
            self.u_hat_history(self.index) = estimated_states(4);
            self.v_hat_history(self.index) = estimated_states(5);
            self.w_hat_history(self.index) = estimated_states(6);
            self.phi_hat_history(self.index) = 180/pi*phi_hat;
            self.theta_hat_history(self.index) = 180/pi*theta_hat;
            self.psi_hat_history(self.index) = 180/pi*psi_hat;
            self.p_hat_history(self.index) = 180/pi*estimated_states(11);
            self.q_hat_history(self.index) = 180/pi*estimated_states(12);
            self.r_hat_history(self.index) = 180/pi*estimated_states(13);
            self.index = self.index + 1;

            % update the plots with associated histories
            % estimated states
            set(self.n_hat_handle, 'Xdata', self.time_history, 'Ydata', self.n_hat_history)
            set(self.e_hat_handle, 'Xdata', self.time_history, 'Ydata', self.e_hat_history)
            set(self.d_hat_handle, 'Xdata', self.time_history, 'Ydata', -self.d_hat_history)
            set(self.u_hat_handle, 'Xdata', self.time_history, 'Ydata', self.u_hat_history)
            set(self.v_hat_handle, 'Xdata', self.time_history, 'Ydata', self.v_hat_history)
            set(self.w_hat_handle, 'Xdata', self.time_history, 'Ydata', self.w_hat_history)
            set(self.phi_hat_handle, 'Xdata', self.time_history, 'Ydata', self.phi_hat_history)
            set(self.theta_hat_handle, 'Xdata', self.time_history, 'Ydata', self.theta_hat_history)
            set(self.psi_hat_handle, 'Xdata', self.time_history, 'Ydata', self.psi_hat_history)
            set(self.p_hat_handle, 'Xdata', self.time_history, 'Ydata', self.p_hat_history)
            set(self.q_hat_handle, 'Xdata', self.time_history, 'Ydata', self.q_hat_history)
            set(self.r_hat_handle, 'Xdata', self.time_history, 'Ydata', self.r_hat_history)
            % desired states
            set(self.n_des_handle, 'Xdata', self.time_history, 'Ydata', self.n_des_history)
            set(self.e_des_handle, 'Xdata', self.time_history, 'Ydata', self.e_des_history)
            set(self.d_des_handle, 'Xdata', self.time_history, 'Ydata', -self.d_des_history)
            set(self.u_des_handle, 'Xdata', self.time_history, 'Ydata', self.u_des_history)
            set(self.v_des_handle, 'Xdata', self.time_history, 'Ydata', self.v_des_history)
            set(self.w_des_handle, 'Xdata', self.time_history, 'Ydata', self.w_des_history)
            set(self.phi_des_handle, 'Xdata', self.time_history, 'Ydata', self.phi_des_history)
            set(self.theta_des_handle, 'Xdata', self.time_history, 'Ydata', self.theta_des_history)
            set(self.psi_des_handle, 'Xdata', self.time_history, 'Ydata', self.psi_des_history)
            set(self.p_des_handle, 'Xdata', self.time_history, 'Ydata', self.p_des_history)
            set(self.q_des_handle, 'Xdata', self.time_history, 'Ydata', self.q_des_history)
            set(self.r_des_handle, 'Xdata', self.time_history, 'Ydata', self.r_des_history)
             % actual states
            set(self.n_handle, 'Xdata', self.time_history, 'Ydata', self.n_history)
            set(self.e_handle, 'Xdata', self.time_history, 'Ydata', self.e_history)
            set(self.d_handle, 'Xdata', self.time_history, 'Ydata', -self.d_history)
            set(self.u_handle, 'Xdata', self.time_history, 'Ydata', self.u_history)
            set(self.v_handle, 'Xdata', self.time_history, 'Ydata', self.v_history)
            set(self.w_handle, 'Xdata', self.time_history, 'Ydata', self.w_history)
            set(self.phi_handle, 'Xdata', self.time_history, 'Ydata', self.phi_history)
            set(self.theta_handle, 'Xdata', self.time_history, 'Ydata', self.theta_history)
            set(self.psi_handle, 'Xdata', self.time_history, 'Ydata', self.psi_history)
            set(self.p_handle, 'Xdata', self.time_history, 'Ydata', self.p_history)
            set(self.q_handle, 'Xdata', self.time_history, 'Ydata', self.q_history)
            set(self.r_handle, 'Xdata', self.time_history, 'Ydata', self.r_history)
            set(self.azimuth_handle, 'Xdata', self.time_history, 'Ydata', self.azimuth_history)
            set(self.elevation_handle, 'Xdata', self.time_history, 'Ydata', self.elevation_history)
            set(self.F_handle, 'Xdata', self.time_history, 'Ydata', self.F_history)
            set(self.tau_x_handle, 'Xdata', self.time_history, 'Ydata', self.tau_x_history)
            set(self.tau_y_handle, 'Xdata', self.time_history, 'Ydata', self.tau_y_history)
            set(self.tau_z_handle, 'Xdata', self.time_history, 'Ydata', self.tau_z_history)
       end
    end
end
