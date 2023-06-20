function R_hand_pose = digit_right_hand_pose(q)
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
    

    

    %%---------------------RIGHT ARM CHAIN--------------------------------
    %27 body to right shoulder roll joint
    T1 = [1 0 0 -0.0010;
               0 1 0 -0.1200;
               0 0 1 0.40;
               0 0 0 1];
    T2 =   [0          0        -1 0;
                -0.173648  0.984808  0 0;
                0.984808  0.1736480  0 0;
                0         0          0 1];

    T3 = [cos(q(27)) -sin(q(27))  0 0;
              sin(q(27)) cos(q(27))   0 0;
              0         0           1 0;
              0         0           0 1];
    Tb2right_shoulder_roll = T1*T2*T3;

         %28 right shoulder roll -> pitch
    T1 = [1 0 0 -0.00317;
               0 1 0 0.011055;
               0 0 1 0.0555;
               0 0 0 1];
    T2 =   [0.679715  -0.679715 -0.275637 0;
                0.194905  -0.194905  0.961262 0;
               -0.707107  -0.707107	 0        0;
                0          0         0        1];

    T3 = [cos(-q(28)) -sin(-q(28))  0 0;
              sin(-q(28)) cos(-q(28))   0 0;
              0           0             1 0;
              0           0             0 1];
    Tright_shoulder_roll2pitch = T1*T2*T3;
    %29 right shoulder pitch -> yaw
    T1 = [1 0 0 0;
               0 1 0 0.165000000000000;
               0 0 1 -0.100000000000000;
               0 0 0 1];
    T2 =   [1	0	0 0;
                0	0	1 0;
                0	-1	0        0;
                0          0         0        1];

    T3 = [cos(q(29)) -sin(q(29))  0 0;
              sin(q(29)) cos(q(29))   0 0;
              0           0             1 0;
              0           0             0 1];
    Tright_shoulder_pitch2yaw = T1*T2*T3;
    %30 right shoulder yaw -> right elbow
    T1 = [1 0 0 0;
               0 1 0 0.0385000000000000;
               0 0 1 0.185000000000000;
               0 0 0 1];
    T2 =   [0.923880000000000	0.382683000000000	0 0;
                0	0	1 0;
                0.382683000000000	-0.923880000000000	0    0;
                0          0         0        1];

    T3 = [cos(q(30)) -sin(q(30))  0 0;
              sin(q(30)) cos(q(30))   0 0;
              0           0             1 0;
              0           0             0 1];
    Tright_shoulder_yaw2elbow = T1*T2*T3;

         %----------arms------
    Tright_elbow2right_fist = [1 0 0 0.24;
                                    0 1 0 -0.1;
                                    0 0 1 0;
                                    0 0 0 1];
    %%--------------------TREES----------------------------

    %RIGHT ELBOW
    q27 = Aw2b *  Tb2right_shoulder_roll;
    q28 = q27 * Tright_shoulder_roll2pitch;
    q29 = q28 * Tright_shoulder_pitch2yaw;
    q30 = q29 * Tright_shoulder_yaw2elbow;


    q30b = q30 * Tright_elbow2right_fist;


    X_R_hand = q30b(1,4);
    Y_R_hand = q30b(2,4);
    Z_R_hand = q30b(3,4);
    Rr21 = q30b(2,1);
    Rr11 = q30b(1,1);
    Rr31 = q30b(3,1);
    Rr32 = q30b(3,2);
    Rr33 = q30b(3,3);
    RPitch = atan2(-Rr31,sqrt(Rr32*Rr32+Rr33*Rr33));
    RYaw = atan2(Rr21,Rr11);
    RRoll = atan2(Rr32,Rr33);
    R_hand_pose = [X_R_hand;Y_R_hand;Z_R_hand];%RRoll;RPitch;RYaw];
    
          
end