function [px_sp2sw, py_sp2sw] = computeStepping(p_sp2CoM, Ly_est, Lx_est, foot_index,LIP_para)
    Ly_des = LIP_para.m * LIP_para.H * LIP_para.v_des;
    [px_sw2CoM, py_sw2CoM] = LIP_planner.computeSw2CoM(Ly_est, Lx_est, Ly_des, foot_index, LIP_para);
    px_sp2CoM = p_sp2CoM(1);
    py_sp2CoM = p_sp2CoM(2);

    px_sp2sw = px_sp2CoM - px_sw2CoM;
    py_sp2sw = py_sp2CoM - py_sw2CoM;
    py_sp2sw = LIP_planner.regulate_lateral_step(foot_index,py_sp2sw);



end