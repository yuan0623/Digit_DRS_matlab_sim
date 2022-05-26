function [u]=feedback_linearization_GPT(t,x,D,c_overall,...
    B_overall,foot_index,current_stance_foot_position,Alpha,t_end_of_previous_step,LIP_para)
    
    q=x(1:30);
    dq=x(31:60);
    % compute hc and its derivative, check section III A
    if foot_index == -1
        hc = output_GPT_func.hc_R_sup(q)-[zeros(14,1);current_stance_foot_position(1);zeros(5,1)];
        j_hc = output_GPT_func.j_hc_R_sup(q);
        jj_hc = output_GPT_func.jj_hc_R_sup(q,dq);
        Alpha_lower=Alpha(1,:);
    elseif foot_index == 1
        hc = output_GPT_func.hc_L_sup(q)-[zeros(14,1);current_stance_foot_position(1);zeros(5,1)];
        j_hc=output_GPT_func.j_hc_L_sup(q);
        jj_hc = output_GPT_func.jj_hc_L_sup(q,dq);
        Alpha_lower=Alpha(2,:);
    end



    theta = q(1)-current_stance_foot_position(1)+ 0.0619;
    dtheta = dq(1);
    ds_dtheta=(Alpha_lower(28)-Alpha_lower(22));
    s = (theta+(0.1))/(0.2)
    ds_dtheta = 5;
    %s=(2*theta-Alpha_lower(22))/ds_dtheta;
    
    
    %%%%%%%%  desired walking pattern  ----->
    is_GPT = true;
    [ph,dph,ddph]=dynamics.digit_new_Bezier_6th(Alpha_lower,s,is_GPT);
    [torso_desired,dtorso_desired,ddtorso_desired]=desired_hip_trajectory(t,t_end_of_previous_step);
    hd = [torso_desired;
            ph(1:4);
            zeros(4,1);
            zeros(4,1);
            ph(5:10)];
    
    dhd = [dtorso_desired;
            dph(1:4);
            zeros(4,1);
            zeros(4,1);
            1/ds_dtheta*dph(5:10)*dtheta];
    ddhd = [ddtorso_desired;
            ddph(1:4);
            zeros(4,1);
            zeros(4,1);
            ddph(5:10)*(dtheta/ds_dtheta)^2];
    pphipq=[ddtorso_desired(1);ddtorso_desired(2);...
            1/ds_dtheta*(ddph*1/ds_dtheta*dtheta^2);
            zeros(8,1)];
        

    %%%%%%%%  desired walking pattern  -----|
    
    %pphipq = [[0;
    %           0;
    %    dph*1/norm_factor*1/sqrt(1-q(1)^2);zeros(8,1)],zeros(20,25)];
    

        
    % y and y_dot are the tracking error, if tracking is good, then they should
    % approaching zero.
    y=hc-hd;
    y_dot=j_hc*dq-dhd;

    norm(y)
    norm(y_dot);

    % make sure that Kp = (Kd/2)^2
    Kp=30^2*eye(20,20);
    Kd=60*eye(20,20);

    v = -Kp*y-Kd*y_dot;
    % check equ. (15)
    %u=(j_hc/(D)*B_overall)\(v+j_hc/(D)*c_overall-jj_hc+ddhd);
    u=((j_hc-pphipq)/(D)*B_overall)\(v+(j_hc-pphipq)/(D)*c_overall-jj_hc+ddhd);
end  

function [torso_desired,dtorso_desired,ddtorso_desired]=desired_hip_trajectory(t,t_end_of_previous_step)
    global t_global 
        

    t_global=[t_global;t+t_end_of_previous_step];

    %a=0.2*sin(pi/8);
    a=0.2;
    %b=-0.01;
    b=0;

    x_torso_desired = b+a*t_global(end);
    y_torso_desired = 0;
    
    
    torso_desired=[x_torso_desired; y_torso_desired];


    dx_torso_desired=a;
    dy_torso_desired=0;
    dtorso_desired=[dx_torso_desired; dy_torso_desired];
    
    ddx_torso_desired=0;
    ddy_torso_desired=0;
    ddtorso_desired=[ddx_torso_desired; ddy_torso_desired];
end