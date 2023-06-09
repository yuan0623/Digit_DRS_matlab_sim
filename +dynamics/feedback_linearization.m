function [u]=feedback_linearization(t,x,D,c_overall,...
    B_overall,foot_index,t_end_of_previous_step,LIP_para,current_stance_foot_position,...
    t_end_desired)
    global Alpha t_global t_LIP_global x0_LIP_sagittal_global x0_LIP_lateral_global...
        u_saittal_global u_lateral_global hc_global hd_global arm_pose_global step_count V_global...
        AM_prediction_global AM_COM_global u_torque_global V_h_global V_eta_global
    persistent t_previous
    if step_count == 1
        LIP_para = LIP_para.initial;

    else
        LIP_para = LIP_para.noninitial;
    end

    q=x(1:30);
    dq=x(31:60);
    %t_global_end = t_global(end);
    %t_ = t_global_end-t_end_desired;
    % compute hc and its derivative, check section III A
    if foot_index == -1  % rightfoot support
        hc = output_func.hc_R_sup(q);
        j_hc = output_func.j_hc_R_sup(q);
        jj_hc = output_func.jj_hc_R_sup(q,dq);
        %swing_foot_pose = forward_kinematics.digit_left_foot_pose(q);
        %u_lateral_star = LIP_para.lateral_LIP.Right.u_star;
        %K_lateral = LIP_para.lateral_LIP.Right.K;
        %y_star_lateral = LIP_para.lateral_LIP.Right.y_star;
        alpha_row_index = 1;
    elseif foot_index == 1 % leftfoot support
        hc = output_func.hc_L_sup(q);
        j_hc=output_func.j_hc_L_sup(q);
        jj_hc = output_func.jj_hc_L_sup(q,dq);
        %swing_foot_pose = forward_kinematics.digit_right_foot_pose(q);
        %u_lateral_star = LIP_para.lateral_LIP.Left.u_star;
        %K_lateral = LIP_para.lateral_LIP.Left.K;
        %y_star_lateral = LIP_para.lateral_LIP.Left.y_star;
        alpha_row_index = 2;
    end
    t_ = t_global(end)-t_end_desired;
    
    %% LIP planner
    if t == 0
        t_previous = 0;
        x0_LIPSagittal = Tool.FOM2LIPSagittal(x,t_,foot_index)
        y0_LIPLateral = Tool.FOM2LIPLateral(x,t_,foot_index)
        %x_star_predict_sagittal = LIP_planner.LIP_run(x0_LIPSagittal,LIP_para,false);
        %y_star_predict_lateral = LIP_planner.LIP_run(y0_LIPLateral,LIP_para,true);

        

            
        %u_sagittal = LIP_para.sagittal_LIP.u_star+LIP_para.sagittal_LIP.K*(x_star_predict_sagittal(1:2)-LIP_para.sagittal_LIP.x_star');
        %u_lateral = u_lateral_star+K_lateral*(y_star_predict_lateral(1:2)-y_star_lateral');

        [u_sagittal, u_lateral] = LIP_planner.getFootPlacement(x0_LIPSagittal, y0_LIPLateral, t_, t_end_desired, LIP_para, foot_index);
        
        %u_lateral = regulate_lateral_step(foot_index,u_lateral);
        %if u_lateral>u_lateral_max
        %   u_lateral = u_lateral_max;
        %elseif u_lateral< u_lateral_min
        %    u_lateral = u_lateral_min;
        %end
        
        
        t_LIP_global = [t_LIP_global,t_global(end)];
        u_saittal_global = [u_saittal_global,u_sagittal];
        u_lateral_global = [u_lateral_global,u_lateral];
        
        Alpha(alpha_row_index,35) = u_sagittal;
        Alpha(alpha_row_index,34) = u_sagittal;
        Alpha(alpha_row_index,42) = u_lateral;
        Alpha(alpha_row_index,41) = u_lateral;
        Alpha(alpha_row_index,30:34) = linspace(Alpha(alpha_row_index,30),Alpha(alpha_row_index,34),5);
        Alpha(alpha_row_index,37:41) = linspace(Alpha(alpha_row_index,37),Alpha(alpha_row_index,41),5);
        
    elseif t < LIP_para.T
        if t-t_previous>0.015
            t_previous = t;
            x0_LIPSagittal = Tool.FOM2LIPSagittal(x,t_,foot_index)
            y0_LIPLateral = Tool.FOM2LIPLateral(x,t_,foot_index)
            %x_star_predict_sagittal = LIP_planner.LIP_run(x0_LIPSagittal,LIP_para,false);
            %y_star_predict_lateral = LIP_planner.LIP_run(y0_LIPLateral,LIP_para,true);
            %u_sagittal = LIP_para.sagittal_LIP.u_star+LIP_para.sagittal_LIP.K*(x_star_predict_sagittal(1:2)-LIP_para.sagittal_LIP.x_star');
            %u_lateral = u_lateral_star+K_lateral*(y_star_predict_lateral(1:2)-y_star_lateral');
            
            [u_sagittal, u_lateral] = LIP_planner.getFootPlacement(x0_LIPSagittal, y0_LIPLateral, t_, t_end_desired, LIP_para, foot_index);
            %u_lateral = regulate_lateral_step(foot_index,u_lateral);
            %if u_lateral>u_lateral_max
            %    u_lateral = u_lateral_max;
            %elseif u_lateral< u_lateral_min
            %    u_lateral = u_lateral_min;
            %end
            
            

            t_LIP_global = [t_LIP_global,t_global(end)];
            u_saittal_global = [u_saittal_global,u_sagittal];
            u_lateral_global = [u_lateral_global,u_lateral];
            
            Alpha(alpha_row_index,35) = u_sagittal;
            Alpha(alpha_row_index,34) = u_sagittal;
            Alpha(alpha_row_index,42) = u_lateral;
            Alpha(alpha_row_index,41) = u_lateral;
            
            Alpha(alpha_row_index,30:34) = linspace(Alpha(alpha_row_index,30),Alpha(alpha_row_index,34),5);
            Alpha(alpha_row_index,37:41) = linspace(Alpha(alpha_row_index,37),Alpha(alpha_row_index,41),5);
            
        end
    end
    %%
    p_com = p_COM_func(q);
    AM_COM = AMworld_about_pA_func(q,dq,p_com);
    AM_COM_global = [AM_COM_global,AM_COM];

    T = LIP_para.T;
    theta = t_;
    dtheta = 1;
    ds_dtheta = 1/T;
    s = theta * ds_dtheta;
    
    
    
    
    %%%%%%%%  desired walking pattern  ----->
    [ph,dph,ddph]=dynamics.digit_new_Bezier_6th(Alpha(alpha_row_index,:),s);
    hd = [ph
          arm_pose_global];
    
    dhd = [ds_dtheta*dph*dtheta;
            zeros(4,1);
            zeros(4,1)];
        
    ddhd = [ddph*(dtheta*ds_dtheta)^2;
            zeros(4,1);
            zeros(4,1)];
    %%%%%%%%  desired walking pattern  -----|
    s
    desired_swing_sagittal = hd(5);
    desired_swing_lateral = hd(6);
    Alpha(alpha_row_index,35);
    Alpha(alpha_row_index,42);
    
    y=hc-hd;
    y_dot=j_hc*dq-dhd;

    norm(y)
    norm(y_dot);
    
    X_h = [y;y_dot];

    t;
    % make sure that Kp = (Kd/2)^2
    Kp=27^2*eye(18,18);
    Kd=54*eye(18,18);
    A = [zeros(18,18),eye(18);-Kp, -Kd];
    Q = eye(36);
    P = lyap(A,Q);
    V_h = X_h'*P*X_h;
    beta = 0.01;
    x0_LIPSagittal = Tool.FOM2LIPSagittal(x,t_,foot_index);
    y0_LIPLateral = Tool.FOM2LIPLateral(x,t_,foot_index);
    X_eta = [x0_LIPSagittal(1:2);y0_LIPLateral(1:2)];
    V_eta = beta*(X_eta'*X_eta);
    V = V_h+V_eta;


    v = -Kp*y-Kd*y_dot;
    % check equ. (15)
    %u=(j_hc/(D)*B_overall)\(v+j_hc/(D)*c_overall-jj_hc+ddhd);
    u=(j_hc/(D)*B_overall)\(v+j_hc/(D)*c_overall-jj_hc+ddhd);


    u_torque_global = [u_torque_global,u];
    hc_global = [hc_global,hc];
    hd_global = [hd_global,hd];
    V_global = [V_global,V];
    V_h_global = [V_h_global,V_h];
    V_eta_global = [V_eta_global, V_eta];
    x0_LIP_sagittal_global = [x0_LIP_sagittal_global,x0_LIPSagittal];
    x0_LIP_lateral_global = [x0_LIP_lateral_global,y0_LIPLateral];
    [Ly_est,Lx_est] = LIP_planner.AMprediction(x0_LIPSagittal, y0_LIPLateral,t,t_end_desired,LIP_para);

    AM_prediction_global = [AM_prediction_global, [Ly_est;Lx_est]];
end  


