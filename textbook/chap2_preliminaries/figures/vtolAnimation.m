classdef vtolAnimation < handle
    %--------------------------------
    properties
        arm_w
        arm_l
        arm_front_x
        arm_front_y
        arm_front_z
        arm_back_x
        arm_back_y
        arm_back_z
        body_forward_x
        body_backward_x
        body_y1
        body_y2
        body_y3
        body_z1
        body_z2
        body_z3
        body_z4
        wing_span
        wing_chord
        wing_depth
        wing_position
        horizontal_tail_span
        horizontal_tail_chord
        horizontal_tail_depth
        horizontal_tail_position
        vertical_tail_span
        vertical_tail_chord
        vertical_tail_depth
        vertical_tail_position
        motor_rad
        motor_height
        rotor_rad
        camera_width
        camera_heigth
        camera_length
        camera_lens
        myred
        mygreen
        myblue
        myyellow
        mygrey1
        mygrey2
        mygrey3
        mygrey4
        vtol_handle
        vertices
        faces
        facecolors
        rotor1_vertices
        rotor1_faces
        rotor1_facecolors
        rotor2_vertices
        rotor2_faces
        rotor2_facecolors
        camera_vertices
        camera_faces
        camera_facecolors
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = vtolAnimation()
            % define physical dimensions of vtol
            self.arm_w = 2/10; % width of arm
            self.arm_l = 15/10; % length of arm
            self.arm_front_x = 6/10; % x position of front arm
            self.arm_front_y = 8/10; % y position of front arm
            self.arm_front_z = 0.8/10; % z position of front arm
            self.arm_back_x = -1/10; % x position of back arm
            self.arm_back_y = 2/10; % y position of back arm
            self.arm_back_z = 0/10; % z position of back arm
            self.body_forward_x = 15/10; %10  % extension forward
            self.body_backward_x = 27/10; %7  % extension backward
            self.body_y1 = 2/10;  % front side-to-side
            self.body_y2 = 3/10;  % middle side-to-side
            self.body_y3 = 1/10;  % rear side-to-side
            self.body_z1 = 0/10;
            self.body_z2 = 2/10;  % depth -up
            self.body_z3 = 4/10; % depth -down
            self.body_z4 = 0/10; % depth back
            self.wing_span = 60/10;
            self.wing_chord = 8/10;
            self.wing_depth = 2/10;
            self.wing_position = [10, 0, 0]/10;
            self.horizontal_tail_span = self.wing_span/3;
            self.horizontal_tail_chord = self.wing_chord/2;
            self.horizontal_tail_depth = self.wing_depth/3;
            self.horizontal_tail_position = [-self.body_backward_x+self.horizontal_tail_chord, 0, 0];
            self.vertical_tail_span = self.horizontal_tail_span/3;
            self.vertical_tail_chord = self.wing_chord/1.5;
            self.vertical_tail_depth = self.wing_depth/2;
            self.vertical_tail_position = [-self.body_backward_x+self.vertical_tail_chord, 0, 0];
            self.motor_rad = self.arm_w/2; % radius of the motor
            self.motor_height = self.arm_w/2; % height of motor
            self.rotor_rad = 8*self.motor_rad;
            self.camera_width = 1.5/10;
            self.camera_heigth = 1/10;
            self.camera_length = 1.5/10; % length of camera
            self.camera_lens = 0.5/10;
            self.myred = [1, 0, 0];
            self.mygreen = [0, 1, 0];
            self.myblue = 0.9*[0, 0, 1];
            self.myyellow = 0.7*[1, 1, 0];
            self.mygrey1 = 0.8*[1, 1, 1];  % light
            self.mygrey2 = 0.6*[1, 1, 1];
            self.mygrey3 = 0.5*[1, 1, 1];
            self.mygrey4 = 0.3*[1, 1, 1]; % dark


            figure(1), clf
            title('self.wing_ed VTOL')
            xlabel('East')
            ylabel('North')
            zlabel('-Down')
            view(32,47)  % set the view angle for figure
            %S = 100;
            axis_size = 5;
            axis([-axis_size,axis_size,-axis_size,axis_size,-axis_size,axis_size]);
            grid off
            
            N  = 20;   % angular resolution for circles

            % define vertices, faces, facecolors for vtol
            [self.vertices, self.faces, self.facecolors]...
                = self.defineVTOL(N);
            [self.rotor1_vertices, self.rotor1_faces, self.rotor1_facecolors]...
                = self.defineRotor(N);
            [self.rotor2_vertices, self.rotor2_faces, self.rotor2_facecolors]...
                = self.defineRotor(N);
            [self.camera_vertices, self.camera_faces, self.camera_facecolors]...
                = self.defineCamera(N);
            self.rotor1_faces = self.rotor1_faces + size(self.vertices,2);
            self.rotor2_faces = self.rotor2_faces + size(self.vertices,2) ...
                + size(self.rotor1_vertices,2);            
            self.camera_faces = self.camera_faces + size(self.vertices,2) ...
                + size(self.rotor1_vertices,2) + size(self.rotor2_vertices,2); 

            % draw vtol at initial conditions
            state = zeros(16,1);
            state(14) = -45 * pi/180;
            state(15) = 45 * pi/180;
            state(16) = 90 * pi/180;
            [V, V_rot1, V_rot2, V_cam] = self.drawVTOL(state);
            self.vtol_handle = patch(...
                    'Vertices', [V'; V_rot1'; V_rot2'; V_cam'],...
                    'Faces', [self.faces; self.rotor1_faces; self.rotor2_faces; self.camera_faces],...
                    'FaceVertexCData',[self.facecolors; self.rotor1_facecolors; self.rotor2_facecolors; self.camera_facecolors],...
                    'FaceColor','flat');
        end
        %---------------------------
        function update(self, x)
            [V, V_rot1, V_rot2, V_cam] = self.drawVTOL(x);
            set(self.vtol_handle, 'Vertices', [V'; V_rot1'; V_rot2'; V_cam']);
            drawnow
        end
        %---------------------------
        function [vertices, rotor1_vertices, rotor2_vertices, camera_vertices] = drawVTOL(self, x)
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
            right_rotor = x(15);     % angle of right rotor (rad)
            left_rotor = x(16);    % angle of left rotor (rad)
                      
            % draws vtol at state x
            vertices = self.rotate(self.vertices, phi, theta, psi); 
            vertices = self.translate(vertices, [pn; pe; pd]);  

            % draw rotor1 and rotor2
            rotor1_attach_point = [self.arm_front_x+self.arm_l; self.arm_front_y+self.wing_span/6; self.arm_front_z];
            rotor1_vertices = self.rotate(self.rotor1_vertices, 0, -pi/2 + right_rotor, 0); 
            rotor1_vertices = self.translate(rotor1_vertices, rotor1_attach_point);  
            rotor1_vertices = self.rotate(rotor1_vertices, phi, theta, psi); 
            rotor1_vertices = self.translate(rotor1_vertices, [pn; pe; pd]);  
            rotor2_attach_point = [self.arm_front_x+self.arm_l; -self.arm_front_y-self.wing_span/6; self.arm_front_z];
            rotor2_vertices = self.rotate(self.rotor2_vertices, 0, -pi/2 + left_rotor, 0); 
            rotor2_vertices = self.translate(rotor2_vertices, rotor2_attach_point);  
            rotor2_vertices = self.rotate(rotor2_vertices, phi, theta, psi); 
            rotor2_vertices = self.translate(rotor2_vertices, [pn; pe; pd]);  

            % draw camera at state x and angle azimuth and elevation
            camera_attach_point = [ 0.5*self.body_forward_x; 0; 1.5*self.body_z2];
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
            rotor1_vertices = R*rotor1_vertices;
            rotor2_vertices = R*rotor2_vertices;
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
        function [V, F, colors] = defineVTOL(self, N)    
            %--------- vertices and faces for center pod ------------
            % vertices of the center pod
            center_vert = [...
                self.body_forward_x,    self.body_y1,  self.body_z1;...    % 1
                self.body_forward_x,    self.body_y1,  self.body_z2;...    % 2
                self.body_forward_x/2,  self.body_y2, -self.body_z2;...    % 3
                self.body_forward_x/2,  self.body_y1,  self.body_z2;...    % 4
                self.body_forward_x/2,  self.body_y1,  self.body_z3;...    % 5
                %-self.body_backward_x,  self.body_y3, -self.body_z2;...    % 6
                -self.body_backward_x,  self.body_y3, 0;...    % 6
                -self.body_backward_x,  self.body_y3,  self.body_z4;...    % 7
                %-self.body_backward_x, -self.body_y3, -self.body_z2;...    % 8
                -self.body_backward_x, -self.body_y3, 0;...    % 8
                -self.body_backward_x, -self.body_y3,  self.body_z4;...    % 9
                self.body_forward_x/2, -self.body_y2, -self.body_z2;...    % 10
                self.body_forward_x/2, -self.body_y1,  self.body_z2;...    % 11
                self.body_forward_x/2, -self.body_y1,  self.body_z3;...    % 12
                self.body_forward_x,   -self.body_y1,  self.body_z1;...    % 13
                self.body_forward_x,   -self.body_y1,  self.body_z2;...    % 14
                ];
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
                self.mygrey1;...    % top
                self.mygrey1;...    % top
                self.mygrey1;...    % right side
                self.mygrey1;...    % right side
                self.mygrey1;...    % left side
                self.mygrey1;...    % left side
                self.mygrey3;...    % back
                self.mygrey2;...    % front top
                self.mygrey2;...    % front bottom
                self.mygrey4;...    % bottom
                self.mygrey4;...    % bottom
                ];
        
            %--------- vertices and faces for main wing ------------
            % vertices of the wing
            wing_vert = [...
                -self.wing_chord,     self.wing_span/2,   0;...                     % 1
                -self.wing_depth/2,  self.wing_span/2,   -self.wing_depth/2;...    % 2
                0,                   self.wing_span/2,   0;...    % 3
                -self.wing_depth/2,  self.wing_span/2,   self.wing_depth/2;...     % 4
                -self.wing_chord,     -self.wing_span/2,   0;...                    % 5
                -self.wing_depth/2,  -self.wing_span/2,   -self.wing_depth/2;...   % 6
                0,                   -self.wing_span/2,   0;...   % 7
                -self.wing_depth/2,  -self.wing_span/2,   self.wing_depth/2;...    % 8
                ];
            wing_vert = wing_vert + repmat(self.wing_position, 8, 1);
            % define faces of wing
            wing_face = [...
                1, 2, 6, 5, 1;...   % top
                1, 4, 8, 5, 1;...   % bottom
                2, 3, 7, 6, 2;...   % top leading edge
                4, 3, 7, 8, 4;...   % bottom leading edge
                1, 2, 3, 4, 1;...   % right side
                5, 6, 7, 8, 5;...   % left side
                ];
            wing_face = [wing_face, repmat(wing_face(:,1),1,2*N-4+2)];
            wing_face = wing_face + size(center_vert,1);
            wing_colors = [...
                self.mygrey1;...    % top
                self.mygrey3;...    % bottom
                self.mygrey1;...    % top leading edge
                self.mygrey1;...    % bottom leading edge
                self.mygrey1;...    % right side
                self.mygrey1;...    % left side
                ];

            %--------- vertices and faces for main horizontal_tail ------------
            % vertices of the horizontal_tail
            horizontal_tail_vert = [...
                -self.horizontal_tail_chord, self.horizontal_tail_span/2, 0;... % 1
                -self.horizontal_tail_depth/2, self.horizontal_tail_span/2, -self.horizontal_tail_depth/2;... % 2
                0, self.horizontal_tail_span/2, 0;... % 3
                -self.horizontal_tail_depth/2, self.horizontal_tail_span/2, self.horizontal_tail_depth/2;... % 4
                -self.horizontal_tail_chord, -self.horizontal_tail_span/2, 0;... % 5
                -self.horizontal_tail_depth/2,  -self.horizontal_tail_span/2,   -self.horizontal_tail_depth/2;...   % 6
                0, -self.horizontal_tail_span/2, 0;... % 7
                -self.horizontal_tail_depth/2, -self.horizontal_tail_span/2, self.horizontal_tail_depth/2;... % 8
                ];
            horizontal_tail_vert = horizontal_tail_vert + repmat(self.horizontal_tail_position, 8, 1);
            % define faces of horizontal_tail
            horizontal_tail_face = [...
                1, 2, 6, 5, 1;...   % top
                1, 4, 8, 5, 1;...   % bottom
                2, 3, 7, 6, 2;...   % top leading edge
                4, 3, 7, 8, 4;...   % bottom leading edge
                1, 2, 4, 4, 1;...   % right side
                5, 6, 7, 8, 5;...   % left side
                ];
            horizontal_tail_face = [horizontal_tail_face, repmat(horizontal_tail_face(:,1),1,2*N-4+2)];
            horizontal_tail_face = horizontal_tail_face + size(center_vert,1) + size(wing_vert,1);
            horizontal_tail_colors = [...
                self.mygrey1;...    % top
                self.mygrey3;...    % bottom
                self.mygrey1;...    % top leading edge
                self.mygrey1;...    % bottom leading edge
                self.mygrey1;...    % right side
                self.mygrey1;...    % left side
                ];

            %--------- vertices and faces for main vertical_tail ------------
            % vertices of the vertical_tail
            vertical_tail_vert = [...
                -self.vertical_tail_chord, self.vertical_tail_span, 0;... % 1
                -self.vertical_tail_chord/2-self.vertical_tail_depth/2, self.vertical_tail_span, -self.vertical_tail_depth/2;... % 2
                -self.vertical_tail_chord/2, self.vertical_tail_span, 0;... % 3
                -self.vertical_tail_chord/2-self.vertical_tail_depth/2, self.vertical_tail_span, self.vertical_tail_depth/2;... % 4
                -self.vertical_tail_chord, 0, 0;... % 5
                -self.vertical_tail_depth/2,  0,   -self.vertical_tail_depth/2;...   % 6
                0, 0, 0;... % 7
                -self.vertical_tail_depth/2, 0, self.vertical_tail_depth/2;... % 8
                ];
            vertical_tail_vert = self.translate(...
                                    self.rotate(vertical_tail_vert', -pi/2, 0, 0),...
                                    self.vertical_tail_position')';
            % define faces of vertical_tail
            vertical_tail_face = [...
                1, 2, 6, 5, 1;...   % top
                1, 4, 8, 5, 1;...   % bottom
                2, 3, 7, 6, 2;...   % top leading edge
                4, 3, 7, 8, 4;...   % bottom leading edge
                1, 2, 4, 4, 1;...   % right side
                5, 6, 7, 8, 5;...   % left side
                ];
            vertical_tail_face = [vertical_tail_face, repmat(vertical_tail_face(:,1),1,2*N-4+2)];
            vertical_tail_face = vertical_tail_face + size(center_vert,1) ...
                + size(wing_vert,1)+  size(horizontal_tail_vert,1);
            vertical_tail_colors = [...
                self.mygrey1;...    % top
                self.mygrey3;...    % bottom
                self.mygrey1;...    % top leading edge
                self.mygrey1;...    % bottom leading edge
                self.mygrey1;...    % right side
                self.mygrey1;...    % left side
                ];

            %--------- vertices and faces for connecting arms ------------  
            arm_vert = [...
                0, self.arm_w/2, -self.arm_w/2;...              % 1
                0, self.arm_w/2, self.arm_w/2;...               % 2
                self.arm_l-self.arm_w/2, self.arm_w/2, -self.arm_w/2;...  % 3
                self.arm_l-self.arm_w/2, self.arm_w/2, 0;...         % 4
                self.arm_l-self.arm_w/2, self.arm_w/2, self.arm_w/2;...   % 5
                self.arm_l, 0.75*self.arm_w/2, 0;...            % 6
                self.arm_l, 0.75*self.arm_w/2, self.arm_w/2;...      % 7
                0, -self.arm_w/2, -self.arm_w/2;...             % 8
                0, -self.arm_w/2, self.arm_w/2;...              % 9
                self.arm_l-self.arm_w/2, -self.arm_w/2, -self.arm_w/2;... % 10
                self.arm_l-self.arm_w/2, -self.arm_w/2, 0;...        % 11
                self.arm_l-self.arm_w/2, -self.arm_w/2, self.arm_w/2;...  % 12
                self.arm_l, -0.75*self.arm_w/2, 0;...           % 13
                self.arm_l, -0.75*self.arm_w/2, self.arm_w/2;...     % 14
                ];
            th = 0;
            arm1_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [self.arm_front_x; self.arm_front_y+self.wing_span/6; self.arm_front_z])';
            th = 0;
            arm2_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [self.arm_front_x; -self.arm_front_y-self.wing_span/6; self.arm_front_z])';
            th = pi;
            arm3_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [self.arm_front_x; self.arm_front_y+self.wing_span/6; self.arm_front_z])';
            th = pi;
            arm4_vert = self.translate(...
                                    self.rotate(arm_vert', 0, 0, th),...
                                    [self.arm_front_x; -self.arm_front_y-self.wing_span/6; self.arm_front_z])';
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
            arm1_face = arm_face + size(center_vert,1) + size(wing_vert,1) ...
                + size(horizontal_tail_vert,1) + size(vertical_tail_vert,1);
            arm2_face = arm1_face + size(arm_vert,1);
            arm3_face = arm2_face + size(arm_vert,1);
            arm4_face = arm3_face + size(arm_vert,1);
            arm_colors = [...
                self.mygrey1;... % top
                self.mygrey1;... % top
                self.mygrey1;... % right side
                self.mygrey1;... % right side
                self.mygrey1;... % left side
                self.mygrey1;... % left side
                self.mygrey4;... % bottom
                self.mygrey4;... % bottom
                self.mygrey2;... % front
                self.mygrey2;... % front
                self.mygrey3;... % back
                ];
    
            %--------- vertices and faces for motors ------------  
%            motor_vert = zeros(22,3);
            [rotor_vert, rotor_face, rotor_colors] = self.defineRotor(N);
            rotor_vert = rotor_vert';
            th = pi;
            rotor3_vert = self.translate(rotor_vert',...
                        [self.arm_front_x+self.arm_l*cos(th); self.arm_front_y+self.arm_l*sin(th)+self.wing_span/6; self.arm_front_z])';
            th = pi;
            rotor4_vert = self.translate(rotor_vert',...
                        [self.arm_front_x+self.arm_l*cos(th); -self.arm_front_y+self.arm_l*sin(th)-self.wing_span/6; self.arm_front_z])';
            M=N/2+1;
            rotor3_face = rotor_face + size(center_vert,1) + size(wing_vert,1) ...
                + size(horizontal_tail_vert,1) + size(vertical_tail_vert,1) + 4*size(arm_vert,1);
            rotor4_face = rotor3_face + size(rotor_vert,1);

            % collect into single [V, F, colors] for vtol
            V = [...
                center_vert;...
                wing_vert;...
                horizontal_tail_vert;...
                vertical_tail_vert;...
                arm1_vert; arm2_vert; arm3_vert; arm4_vert;...
                rotor3_vert; rotor4_vert;...
                ]';
            F = [...
                center_face;...
                wing_face;...
                horizontal_tail_face;...
                vertical_tail_face;...
                arm1_face; arm2_face; arm3_face; arm4_face;...
                rotor3_face; rotor4_face;...
                ];
            colors = [...
                center_colors;... 
                wing_colors;...
                horizontal_tail_colors;...
                vertical_tail_colors;...
                arm_colors; arm_colors; arm_colors; arm_colors;...
                rotor_colors; rotor_colors;...
                ];
        end
        %---------------------------
        function [V, F, colors] = defineRotor(self, N)
            motor_vert = zeros(2*N+2,3);
             for i=0:N
                motor_vert(i+1,:) = [...
                    self.motor_rad*cos(2*pi*i/N),...
                    self.motor_rad*sin(2*pi*i/N),...
                    -self.arm_w];
            end
            for i=N:-1:0
                motor_vert(2*N+2-i,:) = [... 
                    self.motor_rad*cos(2*pi*i/N),...
                    self.motor_rad*sin(2*pi*i/N),...
                    0];
            end
            M=N/2+1;
            motor_face = [...
                [1:N+1, ones(1,N+2)];... % top
                [(N+2)*ones(1,N+2), N+2:2*N+2];... % bottom
                [1:M,(2*N+2-M+1):(2*N+2),(2*N+2)*ones(1,N+1)];... % side 1
                [M:(2*N+2-M+1),(2*N+2-M+1)*ones(1,N+1)];... % side 2
                ];
            motor_colors = [...
                self.mygrey1;... % top
                self.mygrey4;... % bottom
                self.mygrey2;... % side
                self.mygrey2;... % side
                ];

            %--------- vertices and faces for rotors ------------
            rotor_vert = zeros(2*N+2,3);
            for i=0:N
                rotor_vert(i+1,:) = [... 
                    self.rotor_rad*cos(2*pi*i/N),...
                    self.rotor_rad*sin(2*pi*i/N),...
                    -1.1*self.arm_w];
            end
            for i=N:-1:0
                rotor_vert(2*N+2-i,:) = [...
                    self.rotor_rad*cos(2*pi*i/N),...
                    self.rotor_rad*sin(2*pi*i/N),...
                    -1.1*self.arm_w];
            end
            rotor_face = [1:2*N+2,1] + size(motor_vert,1);
            rotor_colors = [...
                self.mygrey1;...
                ];
            % collect into single [V, F, colors] for vtol
            V = [...
                motor_vert;...
                rotor_vert;...
                ]';
            F = [...
                motor_face;...
                rotor_face;...
                ];
            colors = [...
                motor_colors;...
                rotor_colors;...
                ];            
        end

        %---------------------------
        function [V, F, colors] = defineCamera(self, N)
              
            %--------- vertices and faces for camera body ------------  
            camera_vert = [...
                0, self.camera_width/2, -self.camera_heigth/2;...
                0, self.camera_width/2, self.camera_heigth/2;...
                0, -self.camera_width/2, self.camera_heigth/2;...
                0, -self.camera_width/2, -self.camera_heigth/2;...
                self.camera_length, self.camera_width/2, -self.camera_heigth/2;...
                self.camera_length, self.camera_width/2, self.camera_heigth/2;...
                self.camera_length, -self.camera_width/2, self.camera_heigth/2;...
                self.camera_length, -self.camera_width/2, -self.camera_heigth/2;...
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
                self.myblue;... % top
                self.myblue;... % right
                self.myblue;... % bottom
                self.myblue;... % left
                ];

            %--------- vertices and faces for camera lens ------------  
            camera_lens_vert = [...
                self.camera_length+self.camera_lens, self.camera_width/2, -self.camera_heigth/2;...
                self.camera_length+self.camera_lens, self.camera_width/2, self.camera_heigth/2;...
                self.camera_length+self.camera_lens, -self.camera_width/2, self.camera_heigth/2;...
                self.camera_length+self.camera_lens, -self.camera_width/2, -self.camera_heigth/2;...
                ];
                %     
            camera_lens_face = [...
                1, 2, 3, 4, 1;...
                ];
            camera_lens_face = [camera_lens_face,...
                repmat(camera_lens_face(:,1),1,2*N-4+2)];
            camera_lens_face = camera_lens_face + size(camera_vert,1);
            camera_lens_colors = [...
                self.myyellow;...
                ];

            % collect into single [V, F, colors] for vtol
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