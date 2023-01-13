% instantiate animation
animation = quadrotorAnimation();

    phi = 0*pi/180;
    theta = 0*pi/180;
    psi = 45*pi/180;
    azimuth = 0*pi/180;
    elevation = 0*pi/180;
    state = [0; 0; 0; 0; 0; 0; phi; theta; psi; 0; 0; 0; azimuth; elevation];
    animation.update(state);
    view(0,90)
    print -dpng quadrotor_yaw.png
    phi = 45*pi/180;
    theta = 0*pi/180;
    psi = 0*pi/180;
    azimuth = 0*pi/180;
    elevation = 0*pi/180;
    state = [0; 0; 0; 0; 0; 0; phi; theta; psi; 0; 0; 0; azimuth; elevation];
    animation.update(state);
    view(166,0)
    print -dpng quadrotor_roll.png

    phi = 0*pi/180;
    theta = 45*pi/180;
    psi = 0*pi/180;
    azimuth = 0*pi/180;
    elevation = 0*pi/180;
    state = [0; 0; 0; 0; 0; 0; phi; theta; psi; 0; 0; 0; azimuth; elevation];
    animation.update(state);
    view(93,10)
    print -dpng quadrotor_pitch.png

    phi = 0*pi/180;
    theta = 0*pi/180;
    psi = 45*pi/180;
    azimuth = 0*pi/180;
    elevation = 0*pi/180;
    state = [0; 0; 0; 0; 0; 0; phi; theta; psi; 0; 0; 0; azimuth; elevation];
    animation.update(state);
    view(0,90)
    print -dpng quadrotor_yaw.png
    
    phi = 0*pi/180;
    theta = 0*pi/180;
    psi = 0*pi/180;
    azimuth = 45*pi/180;
    elevation = 0*pi/180;
    state = [0; 0; 0; 0; 0; 0; phi; theta; psi; 0; 0; 0; azimuth; elevation];
    animation.update(state);
    view(145,-47)
    print -dpng quadrotor_azimuth.png

    phi = 0*pi/180;
    theta = 0*pi/180;
    psi = 0*pi/180;
    azimuth = 0*pi/180;
    elevation = -45*pi/180;
    state = [0; 0; 0; 0; 0; 0; phi; theta; psi; 0; 0; 0; azimuth; elevation];
    animation.update(state);
    view(95, 2)
    print -dpng quadrotor_elevation.png
