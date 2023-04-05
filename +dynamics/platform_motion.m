function [x_DRS,v_DRS,a_DRS] = platform_motion(t_global,T)
    omega = 2*pi/T;%11.8146; %46.3020;
    x_DRS = zeros(10,1);
    v_DRS = zeros(10,1);
    a_DRS = zeros(10,1);
    amplitude = 0.02;
    x_DRS(2) = amplitude*sin(omega*t_global);
    
    v_DRS(2) = amplitude*omega*cos(omega*t_global);
    a_DRS(2) = -amplitude*omega*omega*sin(omega*t_global);
end