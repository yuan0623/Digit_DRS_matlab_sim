function x0_opt=GetStartingPose(LIP_para,foot_index)
    x0 = rand(30,1);
    A= [];
    b= [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    %AMContact = AMworld_about_pA_func(rand(30,1),rand(30,1),rand(3,1))
    
    sagital_LIP = LIP_para.sagittal_LIP;
    lateral_LIP = LIP_para.lateral_LIP;
    
    
    options = optimoptions('fmincon',...
    'MaxFunEvals',20000,...
    'MaxIter',20000,...
    'ConstraintTolerance',1e-6);
    tic
    [Cost,Cnst] = HLIP_ObjectiveAndConstraints(sagital_LIP,lateral_LIP,foot_index);
    
    [x0_opt,fval,exitflag,output] = fmincon(Cost,x0,A,b,Aeq,beq,...
        lb,ub,Cnst,options);

    toc
end



function [Cost,Cnst] = HLIP_ObjectiveAndConstraints(sagital_LIP,lateral_LIP,foot_index)

    Cost = @obj;
    Cnst = @constr;
    %% cost function
    function J = obj(x0)

        J=(x0(1:30)'-zeros(1,30))*(x0(1:30)-zeros(30,1));

    end

    %% constraints
    function [c,ceq] = constr(x0)
        %% assign the optimization variables
        q0 = x0(1:30);
        CoM = p_COM_func(q0);
        if foot_index == -1
            support_foot = forward_kinematics.digit_right_foot_position(q0);
            support_foot_pose = forward_kinematics.digit_right_foot_pose(q0);
            swing_foot_pose = forward_kinematics.digit_left_foot_pose(q0);
            LIP_lateral = lateral_LIP.Right;
            LIP_lateral.y0(1) = 0.106881;
        elseif foot_index == 1
            support_foot = forward_kinematics.digit_left_foot_position(q0);
            support_foot_pose = forward_kinematics.digit_left_foot_pose(q0);
            swing_foot_pose = forward_kinematics.digit_right_foot_pose(q0);
            LIP_lateral = lateral_LIP.Left;
            LIP_lateral.y0(1) = -0.106881;
        end

        pB2Support = CoM-support_foot;
        
        
        
        
        %% lateral LIP to full order model
        
        ceq1 = LIP_lateral.y0(1) - pB2Support(2);
        ceq2 = [];
        
        %% sagittal LIP to full order model
        ceq3 = [];%sagital_LIP.x0(1) - pB2Support(1);
        ceq4 = [];
        %ceq3 = [];
        %ceq4 = [];
        ceq5 = [];
        %% suppot foot height is zero and flat
        ceq6 = support_foot_pose(3:5);
        %% swing foot is on the ground and flat
        ceq7 = swing_foot_pose(3:5);

        %% CoM height
        ceq8 = CoM(3)-lateral_LIP.H;
        ceq9 = [];
        %% initial step length

        ceq10 = [abs(swing_foot_pose(1)-support_foot_pose(1))-0.0;
            abs(swing_foot_pose(2)-support_foot_pose(2))-0.2347];
        %% heading direction
        ceq11 = [q0(6);
                 support_foot_pose(6);
                 swing_foot_pose(6)];
        %% base orientation
        ceq12 = q0(4:5);
        %% constraint for base config
        c1 = [];
        c2 = -q0(3);
        c3 = abs(q0(4))-0.1;
        c4 = abs(q0(5))-0.1;
        c5 = abs(q0(6))-0.1;
        %% overall constraint
        c = [c1;c2;c3;c4;c5]';
        c = [];

        %ceq = [ceq1;ceq2;ceq3;ceq4;ceq5;ceq6;ceq7;ceq8;ceq9;ceq10]'
        ceq = [ceq1;ceq2;ceq3;ceq4;ceq5;ceq6;ceq7;ceq8;ceq9;ceq10;ceq11;ceq12]'
        
    end
end