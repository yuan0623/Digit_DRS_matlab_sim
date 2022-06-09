function hc = hc_R_sup(q)
    global digit_inertia
    support_foot = forward_kinematics.digit_right_foot_pose(q);
    %tic
    CoM = dynamics.q2CoM(q,digit_inertia.links_offset,digit_inertia.links_mass);
    %toc
    hc = [CoM(1)-support_foot(1);
          CoM(2);
          CoM(3);
          q(4);
          q(5);
          q(6);
          q(15:18);
          q(27:30);
          forward_kinematics.digit_left_foot_pose(q)-[support_foot(1);zeros(5,1)]];
end