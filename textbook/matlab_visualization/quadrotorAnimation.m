classdef quadrotorAnimation < handle
    %--------------------------------
    properties
        quadrotor_handle
        vertices
        faces
        facecolors
        camera_vertices
        camera_faces
        camera_facecolors
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = quadrotorAnimation()
            figure(1), clf
            title('Quadrotor')
            xlabel('East')
            ylabel('North')
            zlabel('-Down')
            view(32,47)  % set the view angle for figure
            %S = 100;
            axis_size = 5;
            axis([-axis_size,axis_size,-axis_size,axis_size,-axis_size,axis_size]);
            grid off
            
            N  = 20;   % angular resolution for circles

            % define vertices, faces, facecolors for quadrotor
            [self.vertices, self.faces, self.facecolors]...
                = self.defineQuadrotor(N);
            [self.camera_vertices, self.camera_faces, self.camera_facecolors]...
                = self.defineCamera(N);
            self.camera_faces = self.camera_faces + size(self.vertices,2);

            % draw quadrotor at initial conditions
            state = zeros(14,1);
            [V, V_cam] = self.drawQuadrotor(state);
            self.quadrotor_handle = patch(...
                    'Vertices', [V'; V_cam'],...
                    'Faces', [self.faces; self.camera_faces],...
                    'FaceVertexCData',[self.facecolors; self.camera_facecolors],...
                    'FaceColor','flat');
        end
        %---------------------------
        function update(self, x)
            [V, V_cam] = self.drawQuadrotor(x);
            set(self.quadrotor_handle, 'Vertices', [V'; V_cam']);
            drawnow
        end
        %---------------------------
        function [vertices, camera_vertices] = drawQuadrotor(self, x)
            % relable states for clarity
            pn        = x(1);       % inertial North position (m)     
            pe        = x(2);       % inertial East position  (m)
            pd        = x(3);       % inertial Down position  (m)
            %u         = x(4);       % body frame velocities (m/2)
            %v         = x(5);       
            %w         = x(6);   
            phi       = x(7);
            theta     = x(8);
            psi       = x(9);
            %p         = x(10);       % roll rate (rad/sec)
            %q         = x(11);       % pitch rate (rad/sec)
            %r         = x(12);       % yaw rate (rad/sec)
            azimuth   = x(13);       % gimbal azimuth angle (rad)
            elevation = x(14);       % gimbal elevation angle (rad)
            
            % define physical dimensions of quadrotor
            body_forward_x = 10;
            body_backward_x = 7;
            body_y1 = 4;
            body_y2 = 5;
            body_y3 = 2;
            body_z1 = 0;
            body_z2 = 2;
            body_z3 = 4;
            body_z4 = 1;
            
            % draws quadrotor at state x
            vertices = self.rotate(self.vertices, phi, theta, psi); 
            vertices = self.translate(vertices, [pn; pe; pd]);  

            % draw camera at state x and angle azimuth and elevation
            camera_attach_point = [ 0.5*body_forward_x; 0; 1.5*body_z2]/10;
            camera_vertices = self.rotate(self.camera_vertices, 0, elevation, azimuth); 
            camera_vertices = self.translate(camera_vertices, camera_attach_point);  
            camera_vertices = self.rotate(camera_vertices, phi, theta, psi); 
            camera_vertices = self.translate(camera_vertices, [pn; pe; pd]);  

            % transform vertices from NED to XYZ (for matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            vertices = R*vertices;
            camera_vertices = R*camera_vertices;
        end
        %---------------------------
        function pts = rotate(self, pts, phi, theta, psi)
            % define pts by euler angle 
            R_roll = [...
                    1, 0, 0;...
                    0, cos(phi), -sin(phi);...
                    0, sin(phi), cos(phi)];
            R_pitch = [...
                    cos(theta), 0, sin(theta);...
                    0, 1, 0;...
                    -sin(theta), 0, cos(theta)];
            R_yaw = [...
                    cos(psi), -sin(psi), 0;...
                    sin(psi), cos(psi), 0;...
                    0, 0, 1];
            R = R_yaw*R_pitch*R_roll;  
            pts = R*pts;
        end  
        %---------------------------
        function pts = translate(self, pts, ned_pos)
            % translate points by ned_pos
            pts = pts + repmat([ned_pos],1,size(pts,2));
        end
        %---------------------------
        function [V, F, colors] = defineQuadrotor(self, N)
            % define physical dimensions of quadrotor
            arm_w = 2/10; % width of arm
            arm_l = 12/10; % length of arm
            arm_front_x = 0.6; % x position of front arm
            arm_front_y = 0.3; % y position of front arm
            arm_front_z = 0.08; % z position of front arm
            arm_back_x = -.1; % x position of back arm
            arm_back_y = 0.2; % y position of back arm
            arm_back_z = 0; % z position of back arm
            body_forward_x = 10;
            body_backward_x = 7;
            body_y1 = 4;
            body_y2 = 5;
            body_y3 = 2;
            body_z1 = 0;
            body_z2 = 2;
            body_z3 = 4;
            body_z4 = 1;
            
            motor_rad = arm_w/2; % radius of the motor
            motor_height = arm_w/4; % height of motor
            rotor_rad = 8*motor_rad; % radius of rotor

            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mygrey1 = 0.8*[1, 1, 1];  % light
            mygrey2 = 0.6*[1, 1, 1];
            mygrey3 = 0.5*[1, 1, 1];
            mygrey4 = 0.3*[1, 1, 1]; % dark
    
            %--------- vertices and faces for center pod ------------
            % vertices of the center pod
            center_vert = [...
                body_forward_x,    body_y1,  body_z1;...    % 1
                body_forward_x,    body_y1,  body_z2;...    % 2
                body_forward_x/2,  body_y2, -body_z2;...    % 3
                body_forward_x/2,  body_y1,  body_z2;...    % 4
                body_forward_x/2,  body_y1,  body_z3;...    % 5
                -body_backward_x,  body_y3, -body_z2;...    % 6
                -body_backward_x,  body_y3,  body_z4;...    % 7
                -body_backward_x, -body_y3, -body_z2;...    % 8
                -body_backward_x, -body_y3,  body_z4;...    % 9
                body_forward_x/2, -body_y2, -body_z2;...    % 10
                body_forward_x/2, -body_y1,  body_z2;...    % 11
                body_forward_x/2, -body_y1,  body_z3;...    % 12
                body_forward_x,   -body_y1,  body_z1;...    % 13
                body_forward_x,   -body_y1,  body_z2;...    % 14
                ]/10;
            % define faces of center pod
            center_face = [...
                1, 3, 10, 13, 1;...   % top
                3, 6, 8, 10, 3;...    % top
                1, 2, 4, 3, 1;...     % right side
                3, 5, 7, 6, 3;...     % right side
                13, 14, 11, 10, 13;... % left side
                10, 8, 9, 12, 10;...   % left side
                8, 9, 7, 6, 8;...     % back
                1, 2, 14, 13, 1;...   % front
                4, 5, 12, 11, 4;...   % front - middle
                2, 4, 11, 14, 2;...   % bottom
                5, 7, 9, 12, 5;...    % bottom
                ];
            center_face = [center_face, repmat(center_face(:,1),1,2*N-4+2)];
            center_colors = [...
                mygrey1;...    % top
                mygrey1;...    % top
                mygrey1;...    % right side
                mygrey1;...    % right side
                mygrey1;...    % left side
                mygrey1;...    % left side
                mygrey3;...    % back
                mygrey2;...    % front top
                mygrey2;...    % front bottom
                mygrey4;...    % bottom
                mygrey4;...    % bottom
                ];
        
    
            %--------- vertices and faces for connecting arms ------------  
            arm_vert = [...
                0, arm_w/2, -arm_w/2;...              % 1
                0, arm_w/2, arm_w/2;...               % 2
                arm_l-arm_w/2, arm_w/2, -arm_w/2;...  % 3
                arm_l-arm_w/2, arm_w/2, 0;...         % 4
                arm_l-arm_w/2, arm_w/2, arm_w/2;...   % 5
                arm_l, 0.75*arm_w/2, 0;...            % 6
                arm_l, 0.75*arm_w/2, arm_w/2;...      % 7
                0, -arm_w/2, -arm_w/2;...             % 8
                0, -arm_w/2, arm_w/2;...              % 9
                arm_l-arm_w/2, -arm_w/2, -arm_w/2;... % 10
                arm_l-arm_w/2, -arm_w/2, 0;...        % 11
                arm_l-arm_w/2, -arm_w/2, arm_w/2;...  % 12
                arm_l, -0.75*arm_w/2, 0;...           % 13
                arm_l, -0.75*arm_w/2, arm_w/2;...     % 14
                ];
            th = pi/4;
            arm1_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [arm_front_x; arm_front_y; arm_front_z])';
            th = -pi/4;
            arm2_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [arm_front_x; -arm_front_y; arm_front_z])';
            th = 3*pi/4;
            arm3_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [arm_back_x; arm_back_y; arm_back_z])';
            th = -3*pi/4;
            arm4_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [arm_back_x; -arm_back_y; arm_back_z])';
            arm_face = [...
                1, 3, 10, 8;...    % top
                4, 6, 13, 11;...   % top
                1, 3, 5, 2;...     % right side
                4, 5, 7, 6;...     % right side
                8, 9, 12, 10;...   % left side
                11, 12, 14, 13;... % left side
                2, 9, 12, 5;...    % bottom
                12, 5, 7, 14;...   % bottom
                13, 14, 7, 6;...   % front
                10, 11, 4, 3;...   % front
                1, 2, 9, 8;...     % back
                ];
            arm_face = [arm_face, repmat(arm_face(:,4),1,2*N-4+3)];
            arm1_face = arm_face + size(center_vert,1);
            arm2_face = arm1_face + size(arm_vert,1);
            arm3_face = arm2_face + size(arm_vert,1);
            arm4_face = arm3_face + size(arm_vert,1);
            arm_colors = [...
                mygrey1;... % top
                mygrey1;... % top
                mygrey1;... % right side
                mygrey1;... % right side
                mygrey1;... % left side
                mygrey1;... % left side
                mygrey4;... % bottom
                mygrey4;... % bottom
                mygrey2;... % front
                mygrey2;... % front
                mygrey3;... % back
                ];
    
            %--------- vertices and faces for motors ------------  
%            motor_vert = zeros(22,3);
            motor_vert = zeros(2*N+2,3);
             for i=0:N
                motor_vert(i+1,:) = [...
                    motor_rad*cos(2*pi*i/N),...
                    motor_rad*sin(2*pi*i/N),...
                    -arm_w];
            end
            for i=N:-1:0
                motor_vert(2*N+2-i,:) = [... 
                    motor_rad*cos(2*pi*i/N),...
                    motor_rad*sin(2*pi*i/N),...
                    0];
            end
            th = pi/4;
            motor1_vert = self.translate(motor_vert',...
                        [arm_front_x+arm_l*cos(th); arm_front_y+arm_l*sin(th); arm_front_z])';
            th = -pi/4;
            motor2_vert = self.translate(motor_vert',...
                        [arm_front_x+arm_l*cos(th); -arm_front_y+arm_l*sin(th); arm_front_z])';
            th = 3*pi/4;
            motor3_vert = self.translate(motor_vert',...
                        [arm_back_x+arm_l*cos(th); arm_back_y+arm_l*sin(th); arm_back_z])';
            th = -3*pi/4;
            motor4_vert = self.translate(motor_vert',...
                        [arm_back_x+arm_l*cos(th); -arm_back_y+arm_l*sin(th); arm_back_z])';
            M=N/2+1;
            motor_face = [...
                [1:N+1, ones(1,N+2)];... % top
                [(N+2)*ones(1,N+2), N+2:2*N+2];... % bottom
%                [1:M,(2*N+2-M+1):(2*N+2),(2*N+2)*ones(1,11)];... % side 1
%                [M:(2*N+2-M+1),(2*N+2-M+1)*ones(1,11)];... % side 2
                [1:M,(2*N+2-M+1):(2*N+2),(2*N+2)*ones(1,N+1)];... % side 1
                [M:(2*N+2-M+1),(2*N+2-M+1)*ones(1,N+1)];... % side 2
                ];
            motor1_face = motor_face + size(center_vert,1) + 4*size(arm_vert,1);
            motor2_face = motor1_face + size(motor_vert,1);
            motor3_face = motor2_face + size(motor_vert,1);
            motor4_face = motor3_face + size(motor_vert,1);
            motor_colors = [...
                mygrey1;... % top
                mygrey4;... % bottom
                mygrey2;... % side
                mygrey2;... % side
                ];

            %--------- vertices and faces for rotors ------------
%            rotor_vert = zeros(22,3);
            rotor_vert = zeros(2*N+2,3);
            for i=0:N
                rotor_vert(i+1,:) = [... 
                    rotor_rad*cos(2*pi*i/N),...
                    rotor_rad*sin(2*pi*i/N),...
                    -1.1*arm_w];
            end
            for i=N:-1:0
                rotor_vert(2*N+2-i,:) = [...
                    rotor_rad*cos(2*pi*i/N),...
                    rotor_rad*sin(2*pi*i/N),...
                    -1.1*arm_w];
            end
            th = pi/4;
            rotor1_vert = self.translate(rotor_vert',...
                        [arm_front_x+arm_l*cos(th); arm_front_y+arm_l*sin(th); arm_front_z])';
            th = -pi/4;
            rotor2_vert = self.translate(rotor_vert',...
                        [arm_front_x+arm_l*cos(th); -arm_front_y+arm_l*sin(th); arm_front_z])';
            th = 3*pi/4;
            rotor3_vert = self.translate(rotor_vert',...
                        [arm_back_x+arm_l*cos(th); arm_back_y+arm_l*sin(th); arm_back_z])';
            th = -3*pi/4;
            rotor4_vert = self.translate(rotor_vert',...
                        [arm_back_x+arm_l*cos(th); -arm_back_y+arm_l*sin(th); arm_back_z])';
            rotor_face = [1:2*N+2,1];
            rotor1_face = rotor_face + size(center_vert,1) + 4*size(arm_vert,1) + 4*size(motor_vert,1);
            rotor2_face = rotor1_face + size(rotor_vert,1);
            rotor3_face = rotor2_face + size(rotor_vert,1);
            rotor4_face = rotor3_face + size(rotor_vert,1);
            rotor_colors = [...
                mygrey1;...
                ];

            % collect into single [V, F, colors] for quadrotor
            V = [...
                center_vert;...
                arm1_vert; arm2_vert; arm3_vert; arm4_vert;...
                motor1_vert; motor2_vert; motor3_vert; motor4_vert;...
                rotor1_vert; rotor2_vert; rotor3_vert; rotor4_vert;...
                ]';
            F = [...
                center_face;...
                arm1_face; arm2_face; arm3_face; arm4_face;...
                motor1_face; motor2_face; motor3_face; motor4_face;...
                rotor1_face; rotor2_face; rotor3_face; rotor4_face;...
                ];
            colors = [...
                center_colors;... 
                arm_colors; arm_colors; arm_colors; arm_colors;...
                motor_colors; motor_colors; motor_colors; motor_colors;...
                rotor_colors; rotor_colors; rotor_colors; rotor_colors;...
                ];
        end
                %---------------------------
        function [V, F, colors] = defineCamera(self, N)
            camera_width = 3/10;
            camera_heigth = 2/10;
            camera_length = 3/10; % length of camera
            camera_lens = 0.5/10;
 
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = 0.9*[0, 0, 1];
            myyellow = 0.7*[1, 1, 0];
            mygrey1 = 0.8*[1, 1, 1];  % light
            mygrey2 = 0.6*[1, 1, 1];
            mygrey3 = 0.5*[1, 1, 1];
            mygrey4 = 0.3*[1, 1, 1]; % dark
            
            %--------- vertices and faces for camera body ------------  
            camera_vert = [...
                0, camera_width/2, -camera_heigth/2;...
                0, camera_width/2, camera_heigth/2;...
                0, -camera_width/2, camera_heigth/2;...
                0, -camera_width/2, -camera_heigth/2;...
                camera_length, camera_width/2, -camera_heigth/2;...
                camera_length, camera_width/2, camera_heigth/2;...
                camera_length, -camera_width/2, camera_heigth/2;...
                camera_length, -camera_width/2, -camera_heigth/2;...
                ];
                %     
            camera_face = [...
                1, 5, 8, 4, 1;... % top
                1, 2, 6, 5, 1;... % right
                2, 3, 7, 6, 2;... % bottom
                3, 4, 8, 7, 3;... % left
                ];
            camera_face = [camera_face, repmat(camera_face(:,1),1,2*N-4+2)];

            camera_colors = [...
                myblue;... % top
                myblue;... % right
                myblue;... % bottom
                myblue;... % left
                ];

            %--------- vertices and faces for camera lens ------------  
            camera_lens_vert = [...
                camera_length+camera_lens, camera_width/2, -camera_heigth/2;...
                camera_length+camera_lens, camera_width/2, camera_heigth/2;...
                camera_length+camera_lens, -camera_width/2, camera_heigth/2;...
                camera_length+camera_lens, -camera_width/2, -camera_heigth/2;...
                ];
                %     
            camera_lens_face = [...
                1, 2, 3, 4, 1;...
                ];
            camera_lens_face = [camera_lens_face,...
                repmat(camera_lens_face(:,1),1,2*N-4+2)];
            camera_lens_face = camera_lens_face + size(camera_vert,1);
            camera_lens_colors = [...
                myyellow;...
                ];

            % collect into single [V, F, colors] for quadrotor
            V = [...
                camera_vert;...
                camera_lens_vert;...
                ]';
            F = [...
                camera_face;...
                camera_lens_face;...
                ];
            colors = [...
                camera_colors;... 
                camera_lens_colors;...
                ];
        end
    end
end