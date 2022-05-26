function hc = hc_R_sup(q)
    hc = [q(3);
          q(4);
          q(5);
          q(6);
          q(15:18);
          q(27:30);
          forward_kinematics.digit_left_foot_pose(q)];
end