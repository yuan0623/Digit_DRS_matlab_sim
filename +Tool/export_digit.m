function export_digit(digit_robot, path)
    for i = 1:length(digit_robot.Mmat)
        M = digit_robot.Mmat{i};
        export(M,path)
    end


    %%
    %X = SymVariable('x',[30,1]);
    %dX = SymVariable('dx',[30,1]);
    X = digit_robot.States.x;
    dX = digit_robot.States.dx;


    pA = SymVariable('p',[3,1]);
    % get terms relatedf to total angular momentum
    AMworld_about_pA = SymExpression(zeros(3,1));

    for j = 1:length(digit_robot.Joints)
        j
        if ~isempty(digit_robot.Joints(i).Actuator)
            T = digit_robot.Joints(i).computeForwardKinematics;
            R = T(1:3,1:3);
            AMworld_about_pA = AMworld_about_pA + R * digit_robot.Joints(i).Axis' * digit_robot.Joints(i).Actuator.RotorInertia * digit_robot.Joints(i).Actuator.GearRatio * dX(i);
        end
    end
    
    for i = 1:length(digit_robot.Links)
        i
        frame = digit_robot.Links(i);
        p = frame.computeCartesianPosition;
        J_p = jacobian(p,X);
        Jb = frame.computeBodyJacobian(length(X));
        T = frame.computeForwardKinematics;
        
        v = J_p*dX;
        
        twist_body_link = Jb*dX;
        w_body_link =  (tomatrix(twist_body_link(4:6)));
        AMbody_link = frame.Inertia*w_body_link;
        AMworld_link = T(1:3,1:3)*AMbody_link;
        
        AMworld_about_pA = AMworld_about_pA + AMworld_link + frame.Mass*cross(p-pA,v);
    end
    
    

    AMworld_about_pA_fun = SymFunction('AMworld_about_pA_func', AMworld_about_pA, {X,dX,pA}, {}, 'true');
    %AMworld_about_pA_fun = SymFunction('AMworld_about_pA_func',AMworld_about_pA,'vars',{X,dX,pA});
    export(AMworld_about_pA_fun, path);

    p_COM= digit_robot.getComPosition()';
    p_COM_fun = SymFunction('p_COM_func',p_COM,{X});
    export(p_COM_fun, path);
end

