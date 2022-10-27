function hc = hc_R_sup(q)
    %global digit_inertia
    support_foot = forward_kinematics.digit_right_foot_pose(q);
    %CoM = dynamics.q2CoM(q,digit_inertia.links_offset,digit_inertia.links_mass);
    CoM = p_COM_func(q);
    hc = [CoM(3);
          q(4);
          q(5);
          q(6);
          %forward_kinematics.digit_left_foot_pose_vehicle(q)];
          forward_kinematics.digit_left_foot_pose(q)-[support_foot(1:2);zeros(4,1)];
          q(15:18);
          q(27:30)];
end