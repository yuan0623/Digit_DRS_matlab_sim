function dx = dynamics(t,x,foot_index,Alpha,current_stance_foot_position,t_end_of_previous_step,LIP_para)

    %global Fr COP
    [D,c_vec,B] = dynamics.dynamic_matrix_digit(x,foot_index);
    q = x(1:30);
    dq = x(31:60);
    if foot_index == -1 % right foot as stance foot
        %j_c = j_Rfoot_pose_func(q);
        j_c = numeric_jacobian(@hol_ctr.right_holonomic_constraint,q);
        jj_c = hol_ctr.jacDotR(q,dq);
    elseif foot_index == 1 % left foot as stance foot
        %j_c = j_Lfoot_pose_func(q);
        j_c = numeric_jacobian(@hol_ctr.left_holonomic_constraint,q);
        jj_c = hol_ctr.jacDotL(q,dq);
    end
    
    c_overall = c_vec-j_c'/(j_c/D*j_c')*(j_c/D*c_vec-jj_c);
    B_overall = B-j_c'/(j_c/D*j_c')*j_c/D*B;
    %digit_left_foot_pose(q)
    %digit_right_foot_pose(q)
    Fx = [dq;(D)\(-c_overall)];
    Gx = [zeros(30,20);(D)\B_overall];
    %u = dynamics.feedback_linearization(t,x,D,c_overall,...
    %B_overall,foot_index,Alpha,t_end_of_previous_step,LIP_para);
    %u=dynamics.feedback_linearization(t,x,D,c_overall,...
    %B_overall,foot_index,Alpha,t_end_of_previous_step,LIP_para);
    u=dynamics.feedback_linearization_GPT(t,x,D,c_overall,...
    B_overall,foot_index,current_stance_foot_position,Alpha,t_end_of_previous_step,LIP_para);
    %u = zeros(20,1);
    dx= Fx+Gx*u;
    F_c = (j_c/D*j_c')\(j_c/D*c_vec-jj_c)-(j_c/D*j_c')\j_c/D*B*u;
    F_r = (j_c/D*j_c')\(j_c/D*c_vec-jj_c)-(j_c/D*j_c')\(j_c/D*B*u);
    %Fr = [Fr,F_r];
    %p_y = F_r(4)/F_r(3);
    %p_x = -F_r(5)/F_r(3);
    %COP = [COP,[p_x;p_y]];
    t;
end