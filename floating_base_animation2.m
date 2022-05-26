function floating_base_animation2(t,x)
    
    figure
    grid on
    axis(gca,'equal')
    num_of_frame=length(t);
    ylim([-1 2]);
    

%     gif('fourtDigitTest.gif','DelayTime',1/60); %add-on 'gif.m' by chad greene
    for i=1:1:num_of_frame
        q = x(i,1:30);
       
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

        figure(1)
        view([0 0]);
        grid on
        axis equal
        xlim([-2 2])
        ylim([-2 2])
        zlim([-2 2])
        hold on
        set(gca,'color','k')
        set(gca,'xcolor','white')
        set(gca,'ycolor','white')
        set(gca,'zcolor','white')
        % set(gcf,'color','#009ADC')
        set(gcf,'color','black')
        % for i = 7:30
        %     txt = "\leftarrow" + num2str(i);
        %     scatter3(eval("q" + num2str(i) + "(1,4)"),eval("q" + num2str(i) + "(2,4)"),eval("q" + num2str(i) + "(3,4)"),'red','filled');
        %     text(eval("q" + num2str(i) + "(1,4)"),eval("q" + num2str(i) + "(2,4)"),eval("q" + num2str(i) + "(3,4)"),txt,'color','white');
        % end
        %%passive joints
        % scatter3(q11(1,4),q11(2,4),q11(3,4),'blue','filled');
        % scatter3(q12(1,4),q12(2,4),q12(3,4),'blue','filled');
        % scatter3(q23(1,4),q23(2,4),q23(3,4),'blue','filled');
        % scatter3(q24(1,4),q24(2,4),q24(3,4),'blue','filled');
        % 
        % scatter3(q18b(1,4),q18b(2,4),q18b(3,4),'red','filled');
        % scatter3(q30b(1,4),q30b(2,4),q30b(3,4),'red','filled');
        % 
        % scatter3(Aw2b(1,4),Aw2b(2,4),Aw2b(3,4),'yellow','filled');

        leftLeg = line([q7(1,4),q8(1,4),q9(1,4),q10(1,4),q11(1,4),q12(1,4),q13(1,4),q14(1,4)], ...
            [q7(2,4),q8(2,4),q9(2,4),q10(2,4),q11(2,4),q12(2,4),q13(2,4),q14(2,4)], ...
            [q7(3,4),q8(3,4),q9(3,4),q10(3,4),q11(3,4),q12(3,4),q13(3,4),q14(3,4)],'color','red','lineWidth',2);
        leftArm = line([q15(1,4),q16(1,4),q17(1,4),q18(1,4),q18b(1,4)], ...
            [q15(2,4),q16(2,4),q17(2,4),q18(2,4),q18b(2,4)], ...
            [q15(3,4),q16(3,4),q17(3,4),q18(3,4),q18b(3,4)],'color','red','lineWidth',2);
        rightLeg = line([q19(1,4),q20(1,4),q21(1,4),q22(1,4),q23(1,4),q24(1,4),q25(1,4),q26(1,4)], ...
            [q19(2,4),q20(2,4),q21(2,4),q22(2,4),q23(2,4),q24(2,4),q25(2,4),q26(2,4)], ...
            [q19(3,4),q20(3,4),q21(3,4),q22(3,4),q23(3,4),q24(3,4),q25(3,4),q26(3,4)],'color','white','lineWidth',2);
        rightArm = line([q27(1,4),q28(1,4),q29(1,4),q30(1,4),q30b(1,4)], ...
            [q27(2,4),q28(2,4),q29(2,4),q30(2,4),q30b(2,4)], ...
            [q27(3,4),q28(3,4),q29(3,4),q30(3,4),q30b(3,4)],'color','white','lineWidth',2);




        pause(0.0005); %use when not making a gif
        %        gif(); %save frame in the gif file

        delete(leftLeg)
        delete(leftArm)
        delete(rightLeg)
        delete(rightArm)
    end
            %figure
            %plot(swing_foot_verticle_overall)
            %title('swing foot position')

end