function [x_DRS,v_DRS,a_DRS] = platform_motion(t_global,T)
    omega = 2*pi/T;%11.8146; %46.3020;
    x_DRS = zeros(10,1);
    v_DRS = zeros(10,1);
    a_DRS = zeros(10,1);
    
    x_DRS(1) = sin(omega*t_global);
    
    v_DRS(1) = omega*cos(omega*t_global);
    a_DRS(1) = -omega*omega*sin(omega*t_global);
end