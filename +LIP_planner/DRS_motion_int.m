function [sum_lateral, sum_sagittal] = DRS_motion_int(T_low,T_high,LIP_para)
    n_int = 500;
    d_tau = (T_high-T_low)/n_int;


    sum_sagittal = zeros(2,1);
    sum_lateral = zeros(2,1);
    for i = 1: n_int
        tau = T_low+(i+1)*d_tau;
        [~, vDRS, ~] = dynamics.platform_motion(tau,LIP_para.T_DRS_x,LIP_para.T_DRS_y,...
            LIP_para.amplitude_x,LIP_para.amplitude_y);

        u_sagittal = [-vDRS(1);0];
        u_lateral = [-vDRS(2);0];

        A_sagittal_t = [0, 1/(LIP_para.m*LIP_para.H);
                        LIP_para.m*LIP_para.g,  0] * (T_high - tau);

        A_lateral_t = [0, -1/(LIP_para.m*LIP_para.H);
                        -LIP_para.m*LIP_para.g,  0]* (T_high - tau);

        sum_sagittal = sum_sagittal + expm(A_sagittal_t)*u_sagittal*d_tau;
        sum_lateral = sum_lateral + expm(A_lateral_t)*u_lateral*d_tau;

    end
    
    %sum_sagittal = zeros(2,1);
    %sum_lateral = zeros(2,1);
    
end