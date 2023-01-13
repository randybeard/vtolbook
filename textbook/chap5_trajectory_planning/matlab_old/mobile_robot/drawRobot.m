function drawRobot(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    V        = uu(3);       % robot forward speed
    psi      = uu(4);       % heading angle  
    pn_d     = uu(5);
    pe_d     = uu(6);
    V_d      = uu(7);
    psi_d    = uu(8);
    t        = uu(9);       % time
    
    % define persistent variables 
    persistent robot_handle;  % figure handle for robot
    persistent desired_handle; % figure handle for desired robot position

    % first time function is called, initialize plot and persistent vars
    if t==0,
        S = 50; % size of plot
        figure(1), clf
        robot_handle = drawBody(pn,pe,psi,[], 'normal','b');
        hold on
        desired_handle = drawBody(pn_d,pe_d,psi_d,[],'normal','r');
        title('Robot')
        xlabel('North')
        ylabel('East')
        axis([-S,S,-S,S]);
        grid on
        
    % at every other time step, redraw quadrotor and target
    else 
        drawBody(pn,pe,psi,robot_handle);
        drawBody(pn_d,pe_d,psi_d,desired_handle);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(pn, pe, psi, handle, mode,color)
  XYpts = robPoints;
  XYpts = rotate(XYpts, psi);  % rotate rigid body  
  XYpts = translate(XYpts, pn, pe);  % translate after rotation

  % transform from N,E to X,Y to render in Matlab
  XYpts = [0, 1; 1, 0]*XYpts;
  
  if isempty(handle),
      handle=fill(XYpts(1,:),XYpts(2,:),color,'EraseMode',mode);
  else
      set(handle,'XData',XYpts(1,:),'YData',XYpts(2,:));
      drawnow
  end
  
end 


%%%%%%%%%%%%%%%%%%%%%%%
function XYpts=rotate(XYpts,psi)
  % define rotation matrix
  R_yaw = [...
          cos(psi), -sin(psi);...
          sin(psi), cos(psi)];

  % rotate vertices
  XYpts  = R_yaw*XYpts;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe
function XYpts = translate(XYpts,pn,pe)

  XYpts = XYpts + repmat([pn;pe],1,size(XYpts,2));
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% robot points
function XYpts = robPoints
  size = .5;
  XYpts = size*[...
    4,4;...
	0,5;...
	-4,2;...
	-4,-2;...
	0,-5;...
    4,-4;...
    4,4;...
  ]';
end
