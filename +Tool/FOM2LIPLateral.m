function x_LIP = FOM2LIPLateral(x,t,foot_index)
    q = x(1:30);
    dq = x(31:60);
    if foot_index == -1
        supportFoot = forward_kinematics.digit_right_foot_pose(q);

    elseif foot_index == 1
        supportFoot = forward_kinematics.digit_left_foot_pose(q);

    end

    COM = p_COM_func(q);
    x_sc = COM(2)-supportFoot(2);
    AM = AMworld_about_pA_func(q,dq,supportFoot(1:3));
    AM_sagittal = AM(1);

    x_LIP = [x_sc;AM_sagittal;t];
    

end