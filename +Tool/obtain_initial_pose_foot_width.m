function q0_opt = obtain_initial_pose_foot_width(q0,foot_index)
    %x0 = rand(1,52);
    A= [];
    b= [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    options = optimoptions('fmincon',...
        'MaxFunEvals',20000,...
        'MaxIter',20000,...
        'ConstraintTolerance',1e-6);
    tic
    [Cost,Cnst] = HLIP_ObjectiveAndConstraints(foot_index,q0);
    
    [q0_opt,fval,exitflag,output] = fmincon(Cost,q0,A,b,Aeq,beq,...
        lb,ub,Cnst,options);

    toc



end

function [Cost,Cnst] = HLIP_ObjectiveAndConstraints(foot_index,q_initial)

    Cost = @obj;
    Cnst = @constr;
    %% cost function
    function J = obj(q0)

        J=(q0'-q_initial')*(q0-q_initial);

    end

    %% constraints
    function [c,ceq] = constr(q0)
        %% assign the optimization variables
       
        desired_step_width = 0.15;
        
        if foot_index == -1
            support_foot_pose = forward_kinematics.digit_right_foot_pose(q0);
            swing_foot_pose = forward_kinematics.digit_left_foot_pose(q0);

            
        elseif foot_index == 1
            support_foot_pose = forward_kinematics.digit_left_foot_pose(q0);
            swing_foot_pose = forward_kinematics.digit_right_foot_pose(q0);

            
        end

        
        
                
        %% y = 0
        
        ceq1 = [];
     

        
        ceq2 = support_foot_pose(1)-swing_foot_pose(1);
        ceq3 = abs(support_foot_pose(2))+abs(swing_foot_pose(2))-desired_step_width;
        ceq4 = [];
        ceq5 = [];
        %% suppot foot height is zero and flat
        ceq6 = support_foot_pose(3:5);
        %% swing foot is on the ground and flat
        ceq7 = swing_foot_pose(3:5);


        %% overall constraint
        %c = [c1;c2;c3;c4;c5]';
        c = [];

        %ceq = [ceq1;ceq2;ceq3;ceq4;ceq5;ceq6;ceq7;ceq8;ceq9;ceq10]'
        ceq = [ceq1;ceq2;ceq3;ceq4;ceq5;ceq6;ceq7]'
        
    end
end