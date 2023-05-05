function [ux, uy] = getFootPlacement(xt,yt,t,LIP_para,foot_index)
    p_sp2com = [xt(1),yt(1)];

    [Ly_est,Lx_est] = LIP_planner.AMprediction(xt,yt,t,LIP_para);
    [ux, uy] = LIP_planner.computeStepping(p_sp2com, Ly_est, Lx_est, foot_index, LIP_para);
end