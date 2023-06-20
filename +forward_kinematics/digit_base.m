function base_pose = digit_base(q)
    % world frame to base frame
    Aw2bp = [1 0 0 q(1);
             0 1 0 q(2);
             0 0 1 q(3);
             0 0 0 1];
    Rb_roll = [1 0          0         0;
               0 cos(q(4)) -sin(q(4)) 0;
               0 sin(q(4))  cos(q(4)) 0;
               0 0          0         1];
    Rb_pitch = [cos(q(5)) 0 sin(q(5)) 0;
                0         1 0         0;
               -sin(q(5)) 0 cos(q(5)) 0;
                0         0 0         1];
    Rb_yaw = [cos(q(6)) -sin(q(6)) 0 0;
              sin(q(6))  cos(q(6)) 0 0;
              0          0         1 0;
              0          0         0 1];
    Rb_rpy = Rb_yaw*Rb_pitch*Rb_roll;
    Aw2b = Aw2bp*Rb_rpy;
    
    base_x = Aw2b(1,4);
    base_y = Aw2b(2,4);
    base_z = Aw2b(3,4);
    
    Lr21 = Aw2b(2,1);

    Lr11 = Aw2b(1,1);

    Lr31 = Aw2b(3,1);

    Lr32 = Aw2b(3,2);

    Lr33 = Aw2b(3,3);

    base_Pitch = atan2(-Lr31,sqrt(Lr32*Lr32+Lr33*Lr33));
    base_Yaw = atan2(Lr21,Lr11);
    base_Roll = atan2(Lr32,Lr33);
    base_pose = [base_x;base_y;base_z];
    
    
          
end