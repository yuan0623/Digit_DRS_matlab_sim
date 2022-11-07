% q(7) hip_abduction_left
% q(8) hip_rotation_left
% q(9) hip_flexion_left
% q(10) knee_joint_left
% q(11) knee_to_shin_left
% q(12) shin_to_tarsus_left
% q(13) toe_pitch_joint_left
% q(14) toe_roll_joint_left

% q(15) shoulder_roll_joint_left
% q(16) shoulder_pitch_joint_left
% q(17) shoulder_yaw_joint_left
% q(18) elbow_joint_left

% q(19) hip_abduction_right
% q(20) hip_rotation_right
% q(21) hip_flextion_right
% q(22) knee_joint_right
% q(23) knee_to_shin_right
% q(24) shin_to_tarsus_right
% q(25) toe_pitch_joint_right
% q(26) toe_roll_joint_right

% q(27) shoulder_roll_joint_right
% q(28) shoulder_pitch_joint_right
% q(29) shoulder_yaw_joint_right
% q(30) elbow_joint_right

clc
clear
tic
global y_global dy_global t_global global_position_reference Alpha t_LIP_global...
    x0_LIP_sagittal_global x0_LIP_lateral_global x_global u_saittal_global u_lateral_global...
    hc_global hd_global arm_pose_global Fr_global
tspan=[0 0.95];
addpath('gen')
%addpath("~/Dropbox/UML_dropbox/Matlab_third_party_package")
% set the intitial condition.
%load initial_pose.mat

%%
foot_index = 1;
load ../LIP_motion_data/Digit/digit_lateral_LIP_v1.mat
load ../LIP_motion_data/Digit/digit_sagittal_LIP_v1.mat
LIP_para.sagittal_LIP = sagittal_LIP;
LIP_para.lateral_LIP = lateral_LIP;
%x0=Tool.LIP2DigitFullModel(LIP_para,foot_index);
%Tool.visualize_IC(x0)
%%  this is to make sure that the initial height of the support foot of the robot is 0.



%x0(31) = v_DRS(1);

%{
x0(31:end)=[0.4975
   -0.6605
    0.7716
   -0.9006
    0.6282
    0.3621
    0.9540
   -0.4309
    0.7184
    0.3852
         0
   -0.3852
    1.6160
    0.0924
   -0.3671
    0.4679
   -0.2832
    0.4434
   -1.9245
   -0.1905
   -0.9559
   -1.3952
         0
    1.3952
   -1.4077
   -1.2738
    0.1486
   -0.9011
   -1.2788
    0.6220]';
%}
%H = 0.7;
%%
load data/x0_v1
arm_pose_global = [x0(15:18);x0(27:30)];
if foot_index == -1
    current_stance_foot_position=forward_kinematics.digit_right_foot_pose(x0(1:30));  %Right foot as stance foot
    swing_foot = forward_kinematics.digit_left_foot_pose(x0(1:30));
elseif foot_index == 1
    current_stance_foot_position=forward_kinematics.digit_left_foot_pose(x0(1:30));  %Left foot as stance foot
    swing_foot = forward_kinematics.digit_right_foot_pose(x0(1:30));
end

Alpha1_R_FD = [1 1 1 1 1 1 1]*lateral_LIP.H;
Alpha2_R_FD = [0,0,0,0,0,0,0];
Alpha3_R_FD = [0,0,0,0,0,0,0];
Alpha4_R_FD = [0,0,0,0,0,0,0];
Alpha5_R_FD = [swing_foot(1)-current_stance_foot_position(1),swing_foot(1),-0.06,0,0.06,0.08,0.25];
Alpha6_R_FD = [swing_foot(2)-current_stance_foot_position(2),swing_foot(2),-0.06,0,0.06,0.08,0.25];
Alpha7_R_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.003]/4;
Alpha8_R_FD = [0,0,0,0,0,0,0];
Alpha9_R_FD = [0,0,0,0,0,0,0];
Alpha10_R_FD = [0,0,0,0,0,0,0];

Alpha1_L_FD = [1 1 1 1 1 1 1]*lateral_LIP.H;
Alpha2_L_FD = [0,0,0,0,0,0,0];
Alpha3_L_FD = [0,0,0,0,0,0,0];
Alpha4_L_FD = [0,0,0,0,0,0,0];
Alpha5_L_FD = [swing_foot(1)-current_stance_foot_position(1),swing_foot(1),-0.06,0,0.06,0.08,0.25];
Alpha6_L_FD = [swing_foot(2)-current_stance_foot_position(2),swing_foot(2),-0.06,0,0.06,0.08,0.25];
Alpha7_L_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.003]/4;
Alpha8_L_FD = [0,0,0,0,0,0,0];
Alpha9_L_FD = [0,0,0,0,0,0,0];
Alpha10_L_FD = [0,0,0,0,0,0,0];

Alpha_R_FD = [Alpha1_R_FD, Alpha2_R_FD, Alpha3_R_FD, Alpha4_R_FD, Alpha5_R_FD, Alpha6_R_FD, Alpha7_R_FD, Alpha8_R_FD, Alpha9_R_FD, Alpha10_R_FD];
Alpha_L_FD = [Alpha1_L_FD, Alpha2_L_FD, Alpha3_L_FD, Alpha4_L_FD, Alpha5_L_FD, Alpha6_L_FD, Alpha7_L_FD, Alpha8_L_FD, Alpha9_L_FD, Alpha10_L_FD];
Alpha = [Alpha_R_FD;Alpha_L_FD];


t=[];
x_sol=[];


t_global=[];
global_position_reference=[];

x0_LIP_sagittal_global = [];
x0_LIP_lateral_global = [];
t_LIP_global = [];
x_global = [];
u_saittal_global = [];
u_lateral_global = [];
hc_global = [];
hd_global = []
Fr_global = [];
step=20;
t_end_of_previous_step=0;

t_interrupt=[];
t_end_desired = 0;
xe_vec = zeros(step,30);


%current_stance_foot_position(1)=0.0376;
%% main loop
for i=1:step
    options=odeset('Events',@(t,x) dynamics.switch_events(t,x,foot_index));

    [t_each_step,x_each_step,te,xe,ie]=ode45(@(t,x) dynamics.dynamics(t,x,...
        foot_index,current_stance_foot_position,t_end_of_previous_step,LIP_para,t_end_desired),tspan,x0,options);
 
    if isempty(t)
        
        t=t_each_step;
        x_sol=x_each_step;
        t_interrupt=te;
        t_end_of_previous_step=t_each_step(end) ;
        stance_foot_position_overall_mat=current_stance_foot_position';
    else
        t_so_far=t_each_step+t(end);
        t_interrupt_each_step=te+t(end);
        t=[t;t_so_far];
        x_sol=[x_sol;x_each_step];
        t_end_of_previous_step=t(end);
        
        t_interrupt=[t_interrupt;t_interrupt_each_step];
    end
    %%
    xe_vec(i,:)= xe(1:30)';
    dq_plus=dynamics.resetmap(x_each_step(end,:),foot_index,LIP_para);
    x0=[x_each_step(end,1:30)';dq_plus];
    
    t_end_desired = LIP_para.sagittal_LIP.T*i;
    
    if foot_index==-1
        current_stance_foot_position=forward_kinematics.digit_right_foot_pose(x0(1:30));
        fprintf('heihei')
        foot_index = 1;
        alpha_row_index = 2;
        hc = output_func.hc_L_sup(x0(1:30));
    elseif foot_index == 1
        current_stance_foot_position=forward_kinematics.digit_left_foot_pose(x0(1:30));
        fprintf('xixi')
        foot_index = -1;
        alpha_row_index = 1;
        hc = output_func.hc_R_sup(x0(1:30));
    end
    Alpha(alpha_row_index,8) = hc(2);
    Alpha(alpha_row_index,15) = hc(3);
    Alpha(alpha_row_index,29) = hc(5);
    Alpha(alpha_row_index,36) = hc(6);
    fprintf('hey,Ive finished %d step\n',i)
    stance_foot_position_overall_mat=[stance_foot_position_overall_mat;current_stance_foot_position'];
end

%% generate the animation
floating_base_animation2(t,x_sol,1,'digit_under_actuation')
%rmpath('gen')
%% generate the tracking results
figure
hold on
plot(t,x_sol(:,1))
%plot(t_global,global_position_reference)
hold off
%legend('actual position', 'reference')

%figure
%hold on
%plot(global_position_reference(2:30:end,1),global_position_reference(2:30:end,2),'--')
%plot(stance_foot_position_overall_mat(:,1),stance_foot_position_overall_mat(:,2),'o')
%hold off

%{
lateral_LIP.H = lateral_LIP_v1.H;
lateral_LIP.T = lateral_LIP_v1.T;
lateral_LIP.Left.u_star = lateral_LIP_v1.Lu_star;
lateral_LIP.Left.y_star = lateral_LIP_v1.Ly_star;
lateral_LIP.Left.K = lateral_LIP_v1.LK;
lateral_LIP.Left.y0 = lateral_LIP_v1.Ly0;

lateral_LIP.Right.u_star = lateral_LIP_v1.Ru_star;
lateral_LIP.Right.y_star = lateral_LIP_v1.Ry_star;
lateral_LIP.Right.K = lateral_LIP_v1.RK;
lateral_LIP.Right.y0 = lateral_LIP_v1.Ry0;
%}