function [Ly_est,Lx_est] = AMprediction(xt,yt,t,t_end_desired,LIP_para)
    global DRS_int_global_tT
    lambda_lip = sqrt(LIP_para.g/LIP_para.H);
    mhl = LIP_para.m * LIP_para.H * lambda_lip;
    px_t = xt(1);
    Ly_t = xt(2);
    py_t = yt(1);
    Lx_t = yt(2);
    T = LIP_para.T;

    if t <= LIP_para.T 
        
    else
        t = T;
    end

    [DRS_motion_int_lateral, DRS_motion_int_sagittal] = LIP_planner.DRS_motion_int(t_end_desired+t,t_end_desired+T, LIP_para);

    DRS_int_global_tT = [DRS_int_global_tT,[DRS_motion_int_lateral; DRS_motion_int_sagittal]];
    Ly_est = mhl * sinh(lambda_lip * (T-t)) * px_t + cosh(lambda_lip * (T-t)) * Ly_t + DRS_motion_int_sagittal(2);
    Lx_est = -mhl * sinh(lambda_lip * (T-t)) * py_t + cosh(lambda_lip * (T-t)) * Lx_t + DRS_motion_int_lateral(2);
end