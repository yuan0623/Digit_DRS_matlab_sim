function dx = dynamics(t,x,LIP_para,is_lateral)
    global t_global
    
    g = 9.81;
    m = LIP_para.sagittal_LIP.m;
    H = LIP_para.sagittal_LIP.H;
    T = LIP_para.sagittal_LIP.T;
    [~,v_DRS,~] = dynamics.platform_motion(t_global(end)+t,T);
    
    %LIP_planner.DRS_motion(t_global(end)+t,T);
    if ~is_lateral
        A = [0 1/(m*H) 0;
            m*g  0 0;
            0    0 0];
        vx_DRS = 0;
    elseif is_lateral
        A = [0 -1/(m*H) 0;
            -m*g  0 0;
            0    0 0];
        vx_DRS = v_DRS(1);
    end
    g_t = [-vx_DRS;
         0;
         1];
    dx = A*x+g_t; 

end