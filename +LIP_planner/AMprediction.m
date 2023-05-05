function [Ly_est,Lx_est] = AMprediction(xt,yt,t,LIP_para)

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
    Ly_est = mhl * sinh(lambda_lip * (T-t)) * px_t + cosh(lambda_lip * (T-t)) * Ly_t;
    Lx_est = -mhl * sinh(lambda_lip * (T-t)) * py_t + cosh(lambda_lip * (T-t)) * Lx_t;
end