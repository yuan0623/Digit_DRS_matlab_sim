function [u]=feedback_linearization(t,x,D,c_overall,...
    B_overall,foot_index,Alpha,t_end_of_previous_step,LIP_para)
    
    q=x(1:30);
    dq=x(31:60);
    % compute hc and its derivative, check section III A
    if foot_index == -1
        hc = output_func.hc_R_sup(q);
        j_hc = output_func.j_hc_R_sup(q);
        jj_hc = output_func.jj_hc_R_sup(q,dq);
        Alpha_lower=Alpha(1,:);
    elseif foot_index == 1
        hc = output_func.hc_L_sup(q);
        j_hc=output_func.j_hc_L_sup(q);
        jj_hc = output_func.jj_hc_L_sup(q,dq);
        Alpha_lower=Alpha(2,:);
    end



    theta = t;
    dtheta = 1;
    s = theta/LIP_para.T
    
    
    %%%%%%%%  desired walking pattern  ----->
    is_GPT = false;
    [ph,dph,ddph]=dynamics.digit_new_Bezier_6th(Alpha_lower,s,is_GPT);
    hd = [LIP_para.H;
            0;
            0;
            0;
            zeros(4,1);
            zeros(4,1);
            ph];
    
    dhd = [ 0;
            0;
            0;
            0;
            zeros(4,1);
            zeros(4,1);
            1/LIP_para.T*dph*dtheta];
    ddhd = [0;
            0;
            0;
            0;
            zeros(4,1);
            zeros(4,1);
            ddph*(dtheta/LIP_para.T)^2];
    
    %%%%%%%%  desired walking pattern  -----|
    
    %pphipq = [[0;
    %           0;
    %    dph*1/norm_factor*1/sqrt(1-q(1)^2);zeros(8,1)],zeros(20,25)];
    

        
    % y and y_dot are the tracking error, if tracking is good, then they should
    % approaching zero.
    y=hc-hd;
    y_dot=j_hc*dq-dhd;

    norm(y);
    norm(y_dot);

    % make sure that Kp = (Kd/2)^2
    Kp=30^2*eye(18,18);
    Kd=60*eye(18,18);

    v = -Kp*y-Kd*y_dot;
    % check equ. (15)
    u=(j_hc/(D)*B_overall)\(v+j_hc/(D)*c_overall-jj_hc+ddhd);
end  