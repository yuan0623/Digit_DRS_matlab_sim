function L_foot_pos = digit_left_foot_position(q)
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
    

    % base to left leg
    Rb2hip_abduction_left1 = [0,0,-1, -1e-03;
                        -0.366501000000000,0.930418000000000,0,0.091;
                        0.930418000000000,0.366501000000000,0,0;
                        0                 0                 0,1];
    Rb2hip_abduction_left2 = [cos(q(7)) -sin(q(7)) 0 0;
                              sin(q(7))  cos(q(7)) 0 0;
                              0          0         1 0;
                              0          0         0 1];
    Rb2hip_abduction_left = Rb2hip_abduction_left1*Rb2hip_abduction_left2;  
    
    Rhip_abduction_left2hip_rotation_left1 = [0 0 -1  -0.0505;
                                              0 1  0  0;
                                              1 0  0  0.0440;
                                              0 0  0  1];
    Rhip_abduction_left2hip_rotation_left2 = [cos(q(8)) -sin(q(8)) 0 0;
                              sin(q(8))  cos(q(8)) 0 0;
                              0          0         1 0;
                              0          0         0 1];  
    Rhip_abduction_left2hip_rotation_left = Rhip_abduction_left2hip_rotation_left1*...
        Rhip_abduction_left2hip_rotation_left2;
    
    Rhip_rotation_left2hip_flexion_left1 = [-0.707107000000000,-0.707107000000000,0,0;
                            0,0,-1,0.004;
                            0.707107000000000,-0.707107000000000,0,0.068;
                            0,                 0,                0, 1];
    Rhip_rotation_left2hip_flexion_left2 = [cos(-q(9)) -sin(-q(9)) 0 0;
                              sin(-q(9))  cos(-q(9)) 0 0;
                              0          0         1 0;
                              0          0         0 1];
    Rhip_rotation_left2hip_flexion_left = Rhip_rotation_left2hip_flexion_left1*...
        Rhip_rotation_left2hip_flexion_left2;
    
    Rhip_flexion_left2knee_joint_left1 = [0 1 0 0.12;
                                        -1 0 0 0;
                                         0 0 1 0.0045;
                                         0 0 0 1];
    Rhip_flexion_left2knee_joint_left2 = [cos(q(10)) -sin(q(10)) 0 0;
                              sin(q(10))  cos(q(10)) 0 0;
                              0          0         1 0;
                              0          0         0 1];                 
    Rhip_flexion_left2knee_joint_left = Rhip_flexion_left2knee_joint_left1*...
        Rhip_flexion_left2knee_joint_left2;
    
    Rknee_joint_left2knee_to_shin_left1 = [1 0 0 0.0607;
                                          0 1 0 0.0474;
                                          0 0 1 0;
                                          0 0 0 1];
    Rknee_joint_left2knee_to_shin_left2 = [cos(q(11)) -sin(q(11)) 0 0;
                              sin(q(11))  cos(q(11)) 0 0;
                              0          0         1 0;
                              0          0         0 1];                                     
    Rknee_joint_left2knee_to_shin_left = Rknee_joint_left2knee_to_shin_left1*...
        Rknee_joint_left2knee_to_shin_left2;
    
    Rknee_to_shin_left2shin_to_tarsus_left1 = [-0.224951000000000,-0.974370000000000,0, 0.4348;
                                                0.974370000000000,-0.224951000000000,0, 0.02;
                                                0,0,1, 0;
                                                0 0 0 1];                                            
    Rknee_to_shin_left2shin_to_tarsus_left2 = [cos(q(12)) -sin(q(12)) 0 0;
                              sin(q(12))  cos(q(12)) 0 0;
                              0          0         1 0;
                              0          0         0 1]; 
                          
    Rknee_to_shin_left2shin_to_tarsus_left = Rknee_to_shin_left2shin_to_tarsus_left1*...
        Rknee_to_shin_left2shin_to_tarsus_left2;
    
    Rshin_to_tarsus_left2toe_pitch_joint_left1 = [0.366455000000000,-0.930436000000000,0, 0.408;
                                    0.930436000000000,0.366455000000000,0,-0.04;
                                    0,0,1,0;
                                    0,0,0,1];
    Rshin_to_tarsus_left2toe_pitch_joint_left2 = [cos(q(13)) -sin(q(13)) 0 0;
                              sin(q(13))  cos(q(13)) 0 0;
                              0          0         1 0;
                              0          0         0 1];    
    Rshin_to_tarsus_left2toe_pitch_joint_left = Rshin_to_tarsus_left2toe_pitch_joint_left1*...
        Rshin_to_tarsus_left2toe_pitch_joint_left2;
    
    Rtoe_pitch_joint_left2toe_roll_joint_left1 = [0,0,1,0;
                                                    0,1,0,0;
                                                    -1,0,0,0;
                                                     0 0 0 1];
    Rtoe_pitch_joint_left2toe_roll_joint_left2 = [cos(q(14)) -sin(q(14)) 0 0;
                              sin(q(14))  cos(q(14)) 0 0;
                              0          0         1 0;
                              0          0         0 1]; 
    Rtoe_pitch_joint_left2toe_roll_joint_left = Rtoe_pitch_joint_left2toe_roll_joint_left1*...
        Rtoe_pitch_joint_left2toe_roll_joint_left2;
    
    Rtoe_roll_joint_left2bottom_feet = [0.0085  0.9990  -0.0443 0;
              -0.4347  0.0436  0.8995 0;
              0.9005  0.0116  0.4346 0;
              0 0 0 1];
          
    Tb2left_toe = Aw2b*Rb2hip_abduction_left*Rhip_abduction_left2hip_rotation_left*...
        Rhip_rotation_left2hip_flexion_left*Rhip_flexion_left2knee_joint_left*...
        Rknee_joint_left2knee_to_shin_left*Rknee_to_shin_left2shin_to_tarsus_left*...
        Rshin_to_tarsus_left2toe_pitch_joint_left*Rtoe_pitch_joint_left2toe_roll_joint_left*...
        Rtoe_roll_joint_left2bottom_feet;
    
    
    
    X_L_foot = Tb2left_toe(1,4);
    X_L_foot = X_L_foot(1);
    Y_L_foot = Tb2left_toe(2,4);
    Y_L_foot = Y_L_foot(1);
    Z_L_foot = Tb2left_toe(3,4);
    Z_L_foot = Z_L_foot(1);

    L_foot_pos = [X_L_foot;Y_L_foot;Z_L_foot];
    
    
          
end