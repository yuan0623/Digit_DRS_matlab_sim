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
    hc_global hd_global arm_pose_global Fr_global step_count DRS_pos_global DRS_int_global_tT DRS_int_global_TT...
    V_global contact_indictor_global AM_prediction_global AM_COM_global u_torque_global ...
    V_h_global V_eta_global
tspan=[0 0.95];
addpath('gen')
addpath('gen_v2')
digit_robot = RobotLinks('digit_model.urdf','floating');
%Tool.export_digit(digit_robot);
%addpath("~/Dropbox/UML_dropbox/Matlab_third_party_package")
% set the intitial condition.
%load initial_pose.mat

%%
foot_index = 1;

%% 
%%%%%

v_des = 0.0;
T_DRS_x = 0.4;
T_DRS_y = 0.72; 
amplitude_x = 0.0;
amplitude_y = 0.06;

LIP_para.initial.T = 0.4;
LIP_para.initial.m = 46.51;
LIP_para.initial.H = 0.9;
LIP_para.initial.W = 0.2;
LIP_para.initial.g = 9.81;
LIP_para.initial.v_des = v_des;
LIP_para.initial.T_DRS_x = T_DRS_x;
LIP_para.initial.T_DRS_y = T_DRS_y;
LIP_para.initial.amplitude_x = amplitude_x;
LIP_para.initial.amplitude_y = amplitude_y;

LIP_para.noninitial.T = 0.4;
LIP_para.noninitial.m = 46.51;
LIP_para.noninitial.H = 0.9;
LIP_para.noninitial.W = 0.2;
LIP_para.noninitial.g = 9.81;
LIP_para.noninitial.v_des = v_des;
LIP_para.noninitial.T_DRS_x = T_DRS_x;
LIP_para.noninitial.T_DRS_y = T_DRS_y;
LIP_para.noninitial.amplitude_x = amplitude_x;
LIP_para.noninitial.amplitude_y = amplitude_y;
%q0=Tool.GetStartingPose(LIP_para.noninitial,foot_index);
%Tool.visualize_IC(q0)
%x0=Tool.LIP2DigitFullModel(LIP_para.noninitial,foot_index);
%Tool.visualize_IC(x0)
%dq0 = zeros(30,1);
%x0 = [q0;dq0];
%%  this is to make sure that the initial height of the support foot of the robot is 0.

q0 = [0.0328437,  -0.01727862,  0.97535971,  0.00576163,  0.00426058,  0.00405035,...
  0.34879835, -0.01270276,  0.22949424,  0.16258706, -0.00076573, -0.16148333,...
  0.06384575, -0.03616253, -0.09930101,  0.89616358, -0.00712672,  0.36241834,...
 -0.34908855, -0.02553236, -0.20902921, -0.18818613,  0.01638354,  0.15432728,...
 -0.02652462,  0.02444792,  0.09914147, -0.89489287,  0.00728998, -0.36258317]';


dq0 = zeros(30,1);
[p_DRS,v_DRS,a_DRS] = dynamics.platform_motion(0,LIP_para.noninitial.T_DRS_x, LIP_para.noninitial.T_DRS_y,...
    LIP_para.noninitial.amplitude_x,LIP_para.noninitial.amplitude_y);
dq0(2) = v_DRS(2);


%%


if foot_index == -1
    current_stance_foot_position=forward_kinematics.digit_right_foot_pose(q0);  %Right foot as stance foot
    swing_foot = forward_kinematics.digit_left_foot_pose(q0);
elseif foot_index == 1
    current_stance_foot_position=forward_kinematics.digit_left_foot_pose(q0);  %Left foot as stance foot
    swing_foot = forward_kinematics.digit_right_foot_pose(q0);
end

q0(3) = q0(3) - current_stance_foot_position(3);
x0 = [q0;dq0];
x0(15) = 0.4;
x0(16) = 0.2;
%x0(17) = 0.2;
x0(18) = 0.4;
x0(27) = -0.4;
x0(28) = -0.2;
%x0(29) = -0.2;
x0(30) = -0.4;
arm_pose_global = [x0(15:18);x0(27:30)];
Alpha1_R_FD = [1 1 1 1 1 1 1]*LIP_para.noninitial.H;
Alpha2_R_FD = [0,0,0,0,0,0,0];
Alpha3_R_FD = [0,0,0,0,0,0,0];
Alpha4_R_FD = [0,0,0,0,0,0,0];
Alpha5_R_FD = [swing_foot(1)-current_stance_foot_position(1),swing_foot(1),-0.06,0,0.06,0.08,0.25];
Alpha6_R_FD = (swing_foot(2)-current_stance_foot_position(2))*[1,1,1,1,1,1,1];
Alpha7_R_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.001]/1.5;
Alpha8_R_FD = [0,0,0,0,0,0,0];
Alpha9_R_FD = [0,0,0,0,0,0,0];
Alpha10_R_FD = [0,0,0,0,0,0,0];

Alpha1_L_FD = [1 1 1 1 1 1 1]*LIP_para.noninitial.H;
Alpha2_L_FD = [0,0,0,0,0,0,0];
Alpha3_L_FD = [0,0,0,0,0,0,0];
Alpha4_L_FD = [0,0,0,0,0,0,0];
Alpha5_L_FD = [swing_foot(1)-current_stance_foot_position(1),swing_foot(1),-0.06,0,0.06,0.08,0.25];
Alpha6_L_FD = (swing_foot(2)-current_stance_foot_position(2))*[1,1,1,1,1,1,1];
Alpha7_L_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.001]/1.5;
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
contact_indictor_global = [];
x0_LIP_sagittal_global = [];
x0_LIP_lateral_global = [];
AM_COM_global = [];
t_LIP_global = [];
x_global = [];
u_saittal_global = [];
u_torque_global = [];
u_lateral_global = [];
hc_global = [];
hd_global = [];
Fr_global = [];
DRS_pos_global = [];
DRS_int_global_tT = [];
DRS_int_global_TT = [];
AM_prediction_global = [];
V_global = [];
V_h_global = [];
V_eta_global = [];
step=20;
t_end_of_previous_step=0;

t_interrupt=[];
t_end_desired = 0;
xe_vec = zeros(step,30);
angular_momentum_offset = 1;

step_count = 1;
%current_stance_foot_position(1)=0.0376;
%% main loop
for i=1:step
    options=odeset('Events',@(t,x) dynamics.switch_events(t,x,foot_index));

    [t_each_step,x_each_step,te,xe,ie]=ode45(@(t,x) dynamics.dynamics(t,x,...
        foot_index,current_stance_foot_position,t_end_of_previous_step,LIP_para,t_end_desired,digit_robot),tspan,x0,options);
 
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
    dq_plus=dynamics.resetmap(x_each_step(end,:),foot_index,LIP_para,digit_robot);
    x0=[x_each_step(end,1:30)';dq_plus];
    
    if step_count == 1
        t_end_desired = LIP_para.initial.T*i;

    else
        t_end_desired = LIP_para.noninitial.T*(i-1)+LIP_para.initial.T;
    end
    
    
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
    Alpha(alpha_row_index,30) = hc(5);
    Alpha(alpha_row_index,36) = hc(6);
    Alpha(alpha_row_index,37) = hc(6);
    fprintf('hey,Ive finished %d step\n',i)
    stance_foot_position_overall_mat=[stance_foot_position_overall_mat;current_stance_foot_position'];
    


    step_count = step_count + 1 ;
end

%% generate the animation
 floating_base_animation2(t,x_sol,0,"digit_UA_type2_planner_DRS_v2")
%rmpath('gen')
%% generate the tracking results
figure
hold on
plot(t,x_sol(:,1))
%plot(t_global,global_position_reference)
hold off
%legend('actual position', 'reference')
