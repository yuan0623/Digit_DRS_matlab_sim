X = SymVariable('x',[30,1]);
dX = SymVariable('dx',[30,1]);
Robot = RobotLinks('digit_model.urdf','floating');
pA = SymVariable('p',[3,1]);
Jq_pA = SymVariable('jqp',[3,length(X)]);
Jdq_pA = SymVariable('jdqp',[3,length(X)]);
% get terms relatedf to total angular momentum
AMworld_about_pA = SymExpression(zeros(3,1));

for i = 1:length(Robot.Links)
    i
    frame = Robot.Links(i);
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

for i = 1:length(Robot.Joints)
    j = i
    if ~isempty(Robot.Joints(i).Actuator)
        T = Robot.Joints(i).computeForwardKinematics;
        R = T(1:3,1:3);
        AMworld_about_pA = AMworld_about_pA + R * Robot.Joints(i).Axis' * Robot.Joints(i).Actuator.Inertia * Robot.Joints(i).Actuator.Ratio * dX(i);
    end
end

Jq_AMworld_about_pA = jacobian(AMworld_about_pA,X);
Jdq_AMworld_about_pA = jacobian(AMworld_about_pA,dX);

Jq_linear = SymExpression(zeros(3,length(X)));
Jdq_linear = SymExpression(zeros(3,length(X)));
for i = 1:length(X)
    Jq_linear(:,i) = frame.Mass*cross(-Jq_pA(:,i),v);
    Jdq_linear(:,i) = frame.Mass*cross(-Jdq_pA(:,i),v);
end
Jq_AMworld_about_pA = Jq_AMworld_about_pA + Jq_linear;
Jdq_AMworld_about_pA = Jdq_AMworld_about_pA + Jdq_linear;

AMworld_about_pA_fun = SymFunction('AMworld_about_pA_func',AMworld_about_pA,{X,dX,pA});
Jq_AMworld_about_pA_fun = SymFunction('Jq_AMworld_about_pA_func',Jq_AMworld_about_pA,{X,dX,pA,Jq_pA});
%Jdq_AMworld_about_pA_fun = SymFunction('Jdq_AMworld_about_pA_func',Jdq_AMworld_about_pA,{X,dX,pA,Jdq_pA});

%export_simulation(AMworld_about_pA,'AMworld_about_pA',EXPO_PATH, {X,dX,pA}, TEMPLATE_PATH);
%export_simulation(Jq_AMworld_about_pA,'Jq_AMworld_about_pA',EXPO_PATH, {X,dX,pA,Jq_pA}, TEMPLATE_PATH);
%export_simulation(Jdq_AMworld_about_pA,'Jdq_AMworld_about_pA',EXPO_PATH, {X,dX,pA,Jdq_pA}, TEMPLATE_PATH);
%%
export(AMworld_about_pA_fun, ' ');
export(Jq_AMworld_about_pA_fun,' ');
export(Jdq_AMworld_about_pA_fun,' ');


export(AMworld_about_pA, 'Vars', {X ,dX,pA}, 'File', ['useless_exe/AMworld_about_pA_SymEx'], 'BuildMex', true, 'ForceExport',true)
%%
% notice: Angular Momentum = f(q)*dq;
p_COM= Robot.getComPosition()';
Jp_COM = jacobian(p_COM,X);
dJp_COM = MatrixTimeDerivative(Jp_COM,X,dX);

p_COM_fun = SymFunction('p_COM_func',p_COM,{X});
Jp_COM_fun = SymFunction('Jp_COM_func',Jp_COM,{X});
dJp_COM_fun = SymFunction('dJp_COM_func',dJp_COM,{X,dX});

export(p_COM_fun, 'gen');
export(Jp_COM_fun, 'gen');
export(dJp_COM_fun, 'gen');

export(SymFunction, 'Vars', {x}, 'File', 'Name', 'TemplateFile', TemplateCpp, 'TemplateHeader', TemplateHpp);

function [dM] = MatrixTimeDerivative(M,X,dX)
dim = size(M);
dM = SymExpression(zeros(dim));
for i = 1:dim(2)
    dM(:,i) = jacobian(M(:,i),X)*dX;
end
end

%%

