% inverse kinematics trial
1+1 ;


hd = [-0.04770363,0.00152742,0.71,0.,0.,0.,0.01,0.01,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,-0.04707125, -0.05516652,0.,0.,0.,0.,0.,0.,0.,0. ]';
paras{1} = 1;
paras{2} = hd;

qqq = fsolve(@(q) virtualConstraint(q,paras), zeros(30,1))
%%
aaa = [-0.0333,  0.0115,  0.7   ,  0.    ,  0.    ,  0.    ,  0.5776, 0.0842, -0.0952, -0.3664,  0.    ,  0.3664, -0.2899,  0.175 , 0.    ,  0.    ,  0.    ,  0.    , -0.4096,  0.0071,  0.0965, 0.4034,  0.    , -0.4034,  0.2722,  0.0417,  0.    ,  0.    ,0.    ,  0.    ];
virtualConstraint(qqq,paras)

function y = virtualConstraint(q,paras)
    foot_index = paras{1};
    if foot_index == 1  % right support
        support_foot = forward_kinematics.digit_right_foot_pose(q);
        swing_foot = forward_kinematics.digit_left_foot_pose(q);
        holonomicCtr = [support_foot; q(11); q(23); q(22)+q(24); q(10)+q(12)];
    elseif foot_index == -1  % left support
        support_foot = forward_kinematics.digit_left_foot_pose(q);
        swing_foot = forward_kinematics.digit_right_foot_pose(q);
        holonomicCtr = [support_foot; q(11); q(23); q(22)+q(24); q(10)+q(12)];
    end
    support_foot(3:end) = 0;
    hd = paras{2};
    CoM = p_COM_func(q);
    hc = [CoM(1);CoM(2);CoM(3);q(4);q(5);q(6);swing_foot-support_foot;q(15:18);q(27:30);holonomicCtr];

    y = hc - hd;
end