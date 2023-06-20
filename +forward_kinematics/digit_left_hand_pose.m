function L_hand_pose = digit_left_hand_pose(q)
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
    

    %%-----------------LEFT ARM CHAIN----------------------------------

    %15 torso -> left shoulder roll
    T1 = [1 0 0 -0.001;
           0 1 0 0.12;
           0 0 1 0.40;
           0 0 0 1];
    T2 = [0	         0       -1 0;
           0.173648  0.984808 0 0;
           0.984808 -0.173648 0 0;
           0         0        0 1];

    T3 = [cos(q(15)) -sin(q(15))  0 0;
          sin(q(15)) cos(q(15))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tb2left_shoulder_roll = T1*T2*T3;
    %16 shoulder roll -> pitch
    T1 = [1 0 0 -0.00317000000000000;
               0 1 0 -0.0110550000000000;
               0 0 1 0.0555000000000000;
               0 0 0 1];
    T2 = [0.679715000000000	0.679715000000000	-0.275637000000000 0;
              -0.194905000000000	-0.194905000000000	-0.961262000000000 0;
               -0.707107000000000	0.707107000000000	0 0;
               0         0        0 1];

    T3 = [cos(-q(16)) -sin(-q(16))  0 0;
              sin(-q(16)) cos(-q(16))   0 0;
              0         0           1 0;
              0         0           0 1];
    Tleft_shoulder_roll2pitch = T1*T2*T3;

    %17 left shoulder pitch -> yaw
    T1 = [1 0 0 0;
               0 1 0 -0.165;
               0 0 1 -0.10;
               0 0 0 1];
    T2 =   [1	0	 0 0;
                0	0	-1 0;
                0	1	 0 0;
                0   0    0 1];

    T3 = [cos(q(17)) -sin(q(17))  0 0;
              sin(q(17)) cos(q(17))   0 0;
              0         0           1 0;
              0         0           0 1];
    Tleft_shoulder_pitch2yaw = T1*T2*T3;
    %18 left shoulder yaw to left elbow joint
    T1 = [1 0 0 0;
               0 1 0 -0.0385;
               0 0 1 0.185;
               0 0 0 1];
    T2 =   [0.92388	-0.382683	 0 0;
                0	     0	        -1 0;
                0.382683	0.92388	 0 0;
                0           0        0 1];

    T3 = [cos(q(18)) -sin(q(18))  0 0;
              sin(q(18)) cos(q(18))   0 0;
              0         0           1 0;
              0         0           0 1];
    Tleft_shoulder_yaw2left_elbow = T1*T2*T3;



    %----------arms------

    Tleft_elbow2left_fist = [1 0 0 0.24;
                             0 1 0 0.1;
                             0 0 1 0;
                             0 0 0 1];
    %%--------------------TREES----------------------------

    %LEFT ELBOW 
    q15 = Aw2b * Tb2left_shoulder_roll;
    q16 = q15 * Tleft_shoulder_roll2pitch;
    q17 = q16 * Tleft_shoulder_pitch2yaw;
    q18 = q17 * Tleft_shoulder_yaw2left_elbow;

    q18b = q18 * Tleft_elbow2left_fist;


    X_L_hand = q18b(1,4);
    Y_L_hand = q18b(2,4);
    Z_L_hand = q18b(3,4);
    Lr21 = q18b(2,1);
    Lr11 = q18b(1,1);
    Lr31 = q18b(3,1);
    Lr32 = q18b(3,2);
    Lr33 = q18b(3,3);
    LPitch = atan2(-Lr31,sqrt(Lr32*Lr32+Lr33*Lr33));
    LYaw = atan2(Lr21,Lr11);
    LRoll = atan2(Lr32,Lr33);
    L_hand_pose = [X_L_hand;Y_L_hand;Z_L_hand];%;LRoll;LPitch;LYaw];
    
          
end