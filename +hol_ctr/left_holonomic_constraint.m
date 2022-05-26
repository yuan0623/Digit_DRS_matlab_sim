function hol_ctr = left_holonomic_constraint(q)
    hol_ctr = [forward_kinematics.digit_left_foot_pose(q);
                q(11);  % rigid_spring
                q(23);  % rigid_spring
                q(22)+q(24);
                q(10)+q(12)];
    

end