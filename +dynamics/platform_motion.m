function [x_DRS,v_DRS,a_DRS] = platform_motion(t_global,T_x,T_y)
    omega_x = 2*pi/T_x;%11.8146; %46.3020;
    omega_y = 2*pi/T_y;

    x_DRS = zeros(10,1);
    v_DRS = zeros(10,1);
    a_DRS = zeros(10,1);


    amplitude_x = 0.1;
    amplitude_y = 0.;

    x_DRS(1) = amplitude_x*cos(omega_x *t_global);
    v_DRS(1) = -amplitude_x*omega_x *sin(omega_x *t_global);
    a_DRS(1) = amplitude_x*omega_x *omega_x *cos(omega_x *t_global);



    x_DRS(2) = amplitude_y*cos(omega_y*t_global);
    v_DRS(2) = -amplitude_y*omega_y*sin(omega_y*t_global);
    a_DRS(2) = amplitude_y*omega_y*omega_y*cos(omega_y*t_global);
end