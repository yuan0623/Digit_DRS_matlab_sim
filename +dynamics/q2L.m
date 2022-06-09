function L = q2L(q,dq,foot_index)
    L_CoM = 0;
    p_CoM = dynamics.q2CoM(q);
    if foot_index == -1
        p_support = forward_kinematics.digit_right_foot_position(q);
    elseif foot_index == 1
        p_support = forward_kinematics.digit_right_foot_position(q);
    end
    m = 46.2;
    p = p_CoM - p_support;
    v_CoM = numeric_jacobian(@dynamics.q2CoM,q)*dq;
    L = L_CoM+cross(p,v_CoM)*m;

end