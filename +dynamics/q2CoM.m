function CoM = q2CoM(q,offset_Digit_links,mass_Digit_links)
    
    %offset_Digit_links = zeros(25,3);
    %mass_Digit_links = ones(25,1);   
    
    %world to base frame
    Aw2bp = [1 0 0 q(1);
             0 1 0 q(2);
             0 0 1 q(3);
             0 0 0 1];
    Tb_roll = [1 0 0 0;
               0 cos(q(4)) -sin(q(4)) 0;
               0 sin(q(4)) cos(q(4)) 0;
               0 0 0 1];
    Tb_pitch = [cos(q(5)) 0 sin(q(5)) 0;
                0 1 0 0;
                -sin(q(5)) 0 cos(q(5)) 0;
                0 0 0 1];
    Tb_yaw = [cos(q(6)) -sin(q(6)) 0 0;
               sin(q(6)) cos(q(6)) 0 0;
               0 0 1 0;
               0 0 0 1];
    Tb2rpy = Tb_roll*Tb_pitch*Tb_yaw;
    Aw2b = Aw2bp * Tb2rpy;

    %%-----------------LEFT LEG CHAIN-------------------------------------

    %7 torso->left hip roll
    T1 = [1 0 0 -0.001;
          0 1 0 0.0910;
          0 0 1 0;
          0 0 0 1];
    T2 = [0	        0	       -1 0;
         -0.366501	0.930418	0 0;
          0.930418	0.366501	0 0;
          0         0           0 1];

    T3 = [cos(q(7)) -sin(q(7))  0 0;
          sin(q(7)) cos(q(7))   0 0;
          0         0           1 0;
          0         0           0 1];

    Tb2left_hip_roll = T1*T2*T3;

    %8 left hip roll -> yaw
    T1 = [1 0 0 -0.0505;
          0 1 0 0;
          0 0 1 0.0440;
          0 0 0 1];
    T2 = [0	0  -1 0;
          0	1	0 0;
          1	0	0 0;
          0 0   0 1];

    T3 = [cos(q(8)) -sin(q(8))  0 0;
          sin(q(8)) cos(q(8))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tleft_hip_roll2yaw = T1*T2*T3;
    %9 left hip yaw -> pitch
    T1 = [1 0 0 0;
       0 1 0 0.0040;
       0 0 1 0.0680;
       0 0 0 1];
    T2 = [-0.707107	-0.707107 0 0;
      0	         0       -1 0;
      0.707107	-0.707107 0 0;
      0          0        0 1];

    T3 = [cos(-q(9)) -sin(-q(9))  0 0;
      sin(-q(9)) cos(-q(9))   0 0;
      0         0           1 0;
      0         0           0 1];

    Tleft_hip_yaw2pitch = T1*T2*T3;
    %10 left hip pitch -> Knee
    T1 = [1 0 0 0.1200;
       0 1 0 0;
       0 0 1 0.0045;
       0 0 0 1];
    T2 = [0	1 0 0;
     -1 0 0 0;
      0	0 1 0;
      0 0 0 1];

    T3 = [cos(q(10)) -sin(q(10))  0 0;
      sin(q(10)) cos(q(10))   0 0;
      0         0           1 0;
      0         0           0 1];

    Tleft_hip_pitch2left_knee = T1*T2*T3;
    %11 left knee -> shin
    T1 = [1 0 0 0.060677;
           0 1 0 0.047406;
           0 0 1 0;
           0 0 0 1];
    T2 = [1	0 0 0;
          0 1 0 0;
          0	0 1 0;
          0 0 0 1];

    T3 = [cos(q(11)) -sin(q(11))  0 0;
          sin(q(11)) cos(q(11))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tleft_knee2left_shin = T1*T2*T3;
    %12  left shin -> tarsus
    T1 = [1 0 0 0.434759;
           0 1 0 0.02;
           0 0 1 0;
           0 0 0 1];
    T2 = [-0.224951	-0.97437  0 0;
           0.97437  -0.224951 0 0;
           0	     0        1 0;
           0         0        0 1];

    T3 = [cos(q(12)) -sin(q(12))  0 0;
          sin(q(12)) cos(q(12))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tleft_shin2left_tarsus = T1*T2*T3;
    %13 left tarsus -> toe pitch
    T1 = [1 0 0 0.408;
           0 1 0 -0.04;
           0 0 1 0;
           0 0 0 1];
    T2 = [0.366455	-0.930436  0 0;
           0.930436  0.366455 0 0;
           0	     0        1 0;
           0         0        0 1];

    T3 = [cos(q(13)) -sin(q(13))  0 0;
          sin(q(13)) cos(q(13))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tleft_tarsus2left_toe_pitch = T1*T2*T3;
    %14 left toe pitch -> toe roll
    T1 =  [1 0 0 0;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];
    T2 = [0	0 1 0;
          0 1 0 0;
         -1 0 0 0;
          0 0 0 1];

    T3 = [cos(q(14)) -sin(q(14))  0 0;
          sin(q(14)) cos(q(14))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tleft_toe_pitch2left_toe_roll = T1*T2*T3;

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


    %%-------------------RIGHT LEG CHAIN----------------------------

    %19 torso -> right hip abduction
    T1 = [1 0 0 -0.001;
           0 1 0 -0.091;
           0 0 1 0;
           0 0 0 1];
    T2 =   [0	      0	       -1 0;
            0.366501  0.930418  0 0;
            0.930418 -0.366501  0 0;
            0         0         0 1];

    T3 = [cos(q(19)) -sin(q(19))  0 0;
          sin(q(19)) cos(q(19))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tb2right_hip_abduction = T1*T2*T3;
    %20 right hip roll -> yaw
    T1 = [1 0 0 -0.0505;
           0 1 0 0;
           0 0 1 0.044;
           0 0 0 1];
    T2 =   [0  0 -1 0;
            0  1  0 0;
            1  0  0 0;
            0  0  0 1];
    T3 = [cos(q(20)) -sin(q(20))  0 0;
          sin(q(20)) cos(q(20))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tright_hip_roll2yaw = T1*T2*T3;
    %21 right hip yaw -> pitch
    T1 = [1 0 0 0;
           0 1 0 -0.004;
           0 0 1 0.068;
           0 0 0 1];
    T2 =   [-0.707107  0.707107  0 0;
            0          0         1 0;
            0.707107   0.707107  0 0;
            0          0         0 1];
    T3 = [cos(-q(21)) -sin(-q(21))  0 0;
          sin(-q(21)) cos(-q(21))   0 0;
          0         0           1 0;
          0         0           0 1];
    Tright_hip_yaw2pitch = T1*T2*T3;
    %22 right_hip_pitch -> right_knee
    T1 = [1 0 0 0.12;
       0 1 0 0;
       0 0 1 0.0045;
       0 0 0 1];
    T2 =   [0  -1 0 0;
        1   0 0 0;
        0   0  1 0;
        0   0  0 1];
    T3 = [cos(q(22)) -sin(q(22))  0 0;
      sin(q(22)) cos(q(22))   0 0;
      0         0           1 0;
      0         0           0 1];
    Tright_hip_pitch2right_knee = T1*T2*T3;
    %23 right knee -> right shin
    T1 = [1 0 0 0.060677;
           0 1 0 -0.0474060;
           0 0 1 0;
           0 0 0 1];
    T2 =   [1  0 0 0;
            0   1 0 0;
            0   0  1 0;
            0   0  0 1];

    T3 = [cos(q(23)) -sin(q(23))  0 0;
          sin(q(23)) cos(q(23))   0 0;
          0         0           1 0;
          0         0           0 1];
     Tright_knee2right_shin = T1*T2*T3;
    %24 right_shin -> right tarsus

    T1 = [1 0 0 0.434759;
       0 1 0 -0.02;
       0 0 1 0;
       0 0 0 1];
    T2 =   [-0.224951  0.974370 0 0;
        -0.974370   -0.2249510 0 0;
        0   0  1 0;
        0   0  0 1];

    T3 = [cos(q(24)) -sin(q(24))  0 0;
      sin(q(24)) cos(q(24))   0 0;
      0         0           1 0;
      0         0           0 1];
    Tright_shin2right_tarsus = T1*T2*T3;
    %25 right_tarsus -> right_toe_pitch

    T1 = [1 0 0 0.4080;
       0 1 0 0.040;
       0 0 1 0;
       0 0 0 1];
    T2 =   [0.366455 0.930436 0 0;
        -0.930436   0.366455 0 0;
        0   0  1 0;
        0   0  0 1];

    T3 = [cos(q(25)) -sin(q(25))  0 0;
      sin(q(25)) cos(q(25))   0 0;
      0         0           1 0;
      0         0           0 1];
    Tright_tarsus2right_toe_pitch = T1*T2*T3;
    %26 right toe pitch -> right toe roll

    T1 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
    T2 =   [0 0 1 0;
        0 1 0 0;
       -1 0 0 0;
        0 0 0 1];

    T3 = [cos(q(26)) -sin(q(26))  0 0;
      sin(q(26)) cos(q(26))   0 0;
      0         0           1 0;
      0         0           0 1];
    Tright_toe_pitch2right_toe_roll = T1*T2*T3;

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
    Tleft_elbow2left_fist = [1 0 0 0.24;
                              0 1 0 0.1;
                              0 0 1 0;
                                0 0 0 1];

    %%--------------------TREES----------------------------
    q1 = Aw2b;
    %LEFT LEG 7->14
    q7 = Aw2b * Tb2left_hip_roll;
    q8 = q7 * Tleft_hip_roll2yaw;
    q9 = q8 * Tleft_hip_yaw2pitch;
    q10 = q9 * Tleft_hip_pitch2left_knee;
    q11 = q10 * Tleft_knee2left_shin;
    q12 = q11 * Tleft_shin2left_tarsus;
    q13 = q12 * Tleft_tarsus2left_toe_pitch;
    q14 = q13 * Tleft_toe_pitch2left_toe_roll;

    %LEFT ELBOW 
    q15 = Aw2b * Tb2left_shoulder_roll;
    q16 = q15 * Tleft_shoulder_roll2pitch;
    q17 = q16 * Tleft_shoulder_pitch2yaw;
    q18 = q17 * Tleft_shoulder_yaw2left_elbow;
    %RIGHT LEG
    q19 = Aw2b * Tb2right_hip_abduction;
    q20 = q19 * Tright_hip_roll2yaw;
    q21 = q20 * Tright_hip_yaw2pitch;
    q22 = q21 * Tright_hip_pitch2right_knee;
    q23 = q22 * Tright_knee2right_shin;
    q24 = q23 *  Tright_shin2right_tarsus;
    q25 = q24 * Tright_tarsus2right_toe_pitch;
    q26 = q25 * Tright_toe_pitch2right_toe_roll;
    %RIGHT ELBOW
    q27 = Aw2b *  Tb2right_shoulder_roll;
    q28 = q27 * Tright_shoulder_roll2pitch;
    q29 = q28 * Tright_shoulder_pitch2yaw;
    q30 = q29 * Tright_shoulder_yaw2elbow;
    q18b = q18 * Tleft_elbow2left_fist;
    q30b = q30 * Tright_elbow2right_fist;
    
    links = {};
    % trunk
    links{1} = q1*[eye(3),offset_Digit_links(1,:)';0,0,0,1];
    % left leg
    links{2} = q7*[eye(3),offset_Digit_links(2,:)';0,0,0,1];
    links{3} = q8*[eye(3),offset_Digit_links(3,:)';0,0,0,1];
    links{4} = q9*[eye(3),offset_Digit_links(4,:)';0,0,0,1];
    links{5} = q10*[eye(3),offset_Digit_links(5,:)';0,0,0,1];
    links{6} = q11*[eye(3),offset_Digit_links(6,:)';0,0,0,1];
    links{7} = q12*[eye(3),offset_Digit_links(7,:)';0,0,0,1];
    links{8} = q13*[eye(3),offset_Digit_links(8,:)';0,0,0,1];
    links{9} = q14*[eye(3),offset_Digit_links(9,:)';0,0,0,1];
    % left eelbow
    links{10} = q15*[eye(3),offset_Digit_links(10,:)';0,0,0,1];
    links{11} = q16*[eye(3),offset_Digit_links(11,:)';0,0,0,1];
    links{12} = q17*[eye(3),offset_Digit_links(12,:)';0,0,0,1];
    links{13} = q18*[eye(3),offset_Digit_links(13,:)';0,0,0,1];
    
    % right leg
    links{14} = q19*[eye(3),offset_Digit_links(14,:)';0,0,0,1];
    links{15} = q20*[eye(3),offset_Digit_links(15,:)';0,0,0,1];
    links{16} = q21*[eye(3),offset_Digit_links(16,:)';0,0,0,1];
    links{17} = q22*[eye(3),offset_Digit_links(17,:)';0,0,0,1];
    links{18} = q23*[eye(3),offset_Digit_links(18,:)';0,0,0,1];
    links{19} = q24*[eye(3),offset_Digit_links(19,:)';0,0,0,1];
    links{20} = q25*[eye(3),offset_Digit_links(20,:)';0,0,0,1];
    links{21} = q26*[eye(3),offset_Digit_links(21,:)';0,0,0,1];
    % right elbow
    links{22} = q27*[eye(3),offset_Digit_links(22,:)';0,0,0,1];
    links{23} = q28*[eye(3),offset_Digit_links(23,:)';0,0,0,1];
    links{24} = q29*[eye(3),offset_Digit_links(24,:)';0,0,0,1];
    links{25} = q30*[eye(3),offset_Digit_links(25,:)';0,0,0,1];
    
    CoM = zeros(3,1);
    for i = 1:length(mass_Digit_links)
        link_T = links{i};
        link_pos = link_T(1:3,4);
        CoM = CoM + link_pos*mass_Digit_links(i);
    end

    CoM = CoM/sum(mass_Digit_links);

end
