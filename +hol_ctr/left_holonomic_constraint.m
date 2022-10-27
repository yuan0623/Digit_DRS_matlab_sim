function hol_ctr = left_holonomic_constraint(q)
    support_foot_constraint = forward_kinematics.digit_left_foot_pose(q);
    hol_ctr = [support_foot_constraint(1:3);
               support_foot_constraint(4:6); 
                q(11);  % rigid_spring
                q(23);  % rigid_spring
                q(22)+q(24);
                q(10)+q(12)];
    

end