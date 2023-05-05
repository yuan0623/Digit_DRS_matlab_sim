function [px_sw2CoM, py_sw2CoM] = computeSw2CoM(Ly_est, Lx_est, Ly_des, foot_index, LIP_para)
    T = LIP_para.T;

    lambda_lip = sqrt(LIP_para.g/LIP_para.H);
    mhl = LIP_para.m * LIP_para.H * lambda_lip;

    alpha = 0.0;
    %DRS_motion_int_lateral, DRS_motion_int_sagittal = self.DRS_motion_int(self.t_begining_current_Step + T,
    %                                                                          self.t_begining_current_Step + 2*T)
    
    px_sw2CoM = (1-alpha) * (Ly_des)/(mhl * sinh(lambda_lip * T)) + (alpha - cosh(lambda_lip*T))/(mhl * sinh(lambda_lip*T)) * Ly_est;
    Lx_des_base = 0.5 * LIP_para.m * LIP_para.H * LIP_para.W * (lambda_lip * sinh(lambda_lip * T)) / (1 + cosh(lambda_lip * T));
    if foot_index == 1 %Left Support
        Lx_des = -Lx_des_base;
    elseif foot_index == -1
        Lx_des = Lx_des_base;
    end
    py_sw2CoM = -(1-alpha) * (Lx_des)/(mhl * sinh(lambda_lip * T)) - (alpha - cosh(lambda_lip*T))/(mhl * sinh(lambda_lip* T)) * Lx_est;

    

end