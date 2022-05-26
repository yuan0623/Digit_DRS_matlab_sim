function R_foot_pose = digit_right_foot_pose(q)
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
    Rb_rpy = Rb_roll*Rb_pitch*Rb_yaw;
    Aw2b = Aw2bp*Rb_rpy;
    
    % base to right leg

    Rb2hip_abduction_right1 = [0,0,-1,-1e-3;
                        0.366501000000000,0.930418000000000,0,-0.091;
                        0.930418000000000,-0.366501000000000,0,0;
                        0                 0                  0,1];
                    
    Rb2hip_abduction_right2 = [cos(q(19)) -sin(q(19)) 0 0;
                              sin(q(19))  cos(q(19)) 0 0;
                              0          0         1 0;
                              0          0         0 1];
    Rb2hip_abduction_right = Rb2hip_abduction_right1*...
        Rb2hip_abduction_right2;
    
    Rhip_abduction_right2hip_rotation_right1 =[0,0,-1,-0.0505;
                                            0,1,0,0;
                                             1,0,0,0.044;
                                             0,0,0,1];
                                         
    Rhip_abduction_right2hip_rotation_right2 = [cos(q(20)) -sin(q(20)) 0 0;
                              sin(q(20))  cos(q(20)) 0 0;
                              0          0         1 0;
                              0          0         0 1];
    Rhip_abduction_right2hip_rotation_right = Rhip_abduction_right2hip_rotation_right1*...
        Rhip_abduction_right2hip_rotation_right2;
    
    Rhip_rotation_right2hip_flexion_right1 = [-0.707107000000000,0.707107000000000,0,0;
                                        0,0,1,-0.004;
                                        0.707107000000000,0.707107000000000,0,0.068;
                                        0,0,0,1];
    Rhip_rotation_right2hip_flexion_right2 = [cos(-q(21)) -sin(-q(21)) 0 0;
                              sin(-q(21))  cos(-q(21)) 0 0;
                              0          0         1 0;
                              0          0         0 1];
    Rhip_rotation_right2hip_flexion_right = Rhip_rotation_right2hip_flexion_right1*...
        Rhip_rotation_right2hip_flexion_right2;
    
    Rhip_flexion_right2knee_joint_right1 = [0,-1,0,0.12;
                                        1,0,0,0;
                                        0,0,1,0.0045;
                                        0,0,0,1];
    Rhip_flexion_right2knee_joint_right2 = [cos(q(22)) -sin(q(22)) 0 0;
                              sin(q(22))  cos(q(22)) 0 0;
                              0          0         1 0;
                              0          0         0 1];  
    Rhip_flexion_right2knee_joint_right = Rhip_flexion_right2knee_joint_right1*...
        Rhip_flexion_right2knee_joint_right2;
    
    Rknee_joint_right2knee_to_shin_right1 = [1,0,0,0.0607;
                                            0,1,0,-0.0474;
                                            0,0,1,0;
                                            0,0,0,1];
    Rknee_joint_right2knee_to_shin_right2 =  [cos(q(23)) -sin(q(23)) 0 0;
                              sin(q(23))  cos(q(23)) 0 0;
                              0          0         1 0;
                              0          0         0 1];   
    Rknee_joint_right2knee_to_shin_right = Rknee_joint_right2knee_to_shin_right1*...
        Rknee_joint_right2knee_to_shin_right2;
    
    Rknee_to_shin_right2shin_to_tarsus_right1 = [-0.224951000000000,0.974370000000000,0,0.4348;
                                        -0.974370000000000,-0.224951000000000,0,-0.02;
                                         0,0,1,0;
                                         0,0,0,1];
    Rknee_to_shin_right2shin_to_tarsus_right2 = [cos(q(24)) -sin(q(24)) 0 0;
                              sin(q(24))  cos(q(24)) 0 0;
                              0          0         1 0;
                              0          0         0 1];
    Rknee_to_shin_right2shin_to_tarsus_right = Rknee_to_shin_right2shin_to_tarsus_right1*...
        Rknee_to_shin_right2shin_to_tarsus_right2;
    
    Rshin_to_tarsus_right2toe_pitchi_joint_right1 = [0.366455000000000,0.930436000000000,0,0.408;
                                            -0.930436000000000,0.366455000000000,0,0.04;
                                                0,0,1,0;
                                                0,0,0,1];
    Rshin_to_tarsus_right2toe_pitchi_joint_right2 = [cos(q(25)) -sin(q(25)) 0 0;
                              sin(q(25))  cos(q(25)) 0 0;
                              0          0         1 0;
                              0          0         0 1];  
    Rshin_to_tarsus_right2toe_pitchi_joint_right = Rshin_to_tarsus_right2toe_pitchi_joint_right1*...
        Rshin_to_tarsus_right2toe_pitchi_joint_right2;
    
    Rtoe_pitchi_joint_right2toe_roll_joint_right1 = [0,0,1,0;
                                                0,1,0,0;
                                                -1,0,0,0;
                                                0,0,0,1];
    Rtoe_pitchi_joint_right2toe_roll_joint_right2 = [cos(q(26)) -sin(q(26)) 0 0;
                              sin(q(26))  cos(q(26)) 0 0;
                              0          0         1 0;
                              0          0         0 1];  
    Rtoe_pitchi_joint_right2toe_roll_joint_right = Rtoe_pitchi_joint_right2toe_roll_joint_right1*...
        Rtoe_pitchi_joint_right2toe_roll_joint_right2;
    
    Rtoe_roll_joint_right2bottom_feet = [0.0115  -0.9974  -0.0716 0;
                    0.4451  0.0692  -0.8928 0;
                    0.8954  -0.0216  0.4448 0;
                    0 0 0 1];

    Tb2right_toe = Aw2b*Rb2hip_abduction_right*Rhip_abduction_right2hip_rotation_right*...
        Rhip_rotation_right2hip_flexion_right*Rhip_flexion_right2knee_joint_right*...
        Rknee_joint_right2knee_to_shin_right*Rknee_to_shin_right2shin_to_tarsus_right*...
        Rshin_to_tarsus_right2toe_pitchi_joint_right*Rtoe_pitchi_joint_right2toe_roll_joint_right*...
        Rtoe_roll_joint_right2bottom_feet;
    
    X_R_foot = Tb2right_toe(1,4);
    X_R_foot = X_R_foot(1);
    Y_R_foot = Tb2right_toe(2,4);
    Y_R_foot = Y_R_foot(1);
    Z_R_foot = Tb2right_toe(3,4);
    Z_R_foot = Z_R_foot(1);
    Rr21 = Tb2right_toe(2,1);
    Rr21 = Rr21(1);
    Rr11 = Tb2right_toe(1,1);
    Rr11 = Rr11(1);
    Rr31 = Tb2right_toe(3,1);
    Rr31 = Rr31(1);
    Rr32 = Tb2right_toe(3,2);
    Rr32 = Rr32(1);
    Rr33 = Tb2right_toe(3,3);
    Rr33 = Rr33(1);
    RPitch = atan2(-Rr31,sqrt(Rr32*Rr32+Rr33*Rr33));
    RYaw = atan2(Rr21,Rr11);
    RRoll = atan2(Rr32,Rr33);
    R_foot_pose = [X_R_foot;Y_R_foot;Z_R_foot;RRoll;RPitch;RYaw];

    
    
          
end