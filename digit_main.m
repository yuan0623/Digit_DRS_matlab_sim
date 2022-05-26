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
global y_global dy_global t_global global_position_reference 
tspan=[0 7];
addpath('gen')
%addpath("~/Dropbox/UML_dropbox/Matlab_third_party_package")
% set the intitial condition.
load initial_pose.mat
x0=zeros(1,60);
%x0(1:30) = joint_angle';


%Fr = [];
%COP = zeros(2,1);

%x0(3) = 0.2876;
%%  this is to make sure that the initial height of the support foot of the robot is 0.
a=forward_kinematics.digit_left_foot_pose(x0(1:30));
y_global=[];

x0(3)=-a(3);
dy_global=[];
phi_global=[];

%% set the Bezier curve coefficient, this is used to parameterize the walking pattern.
%{
Alpha1_R_FD = linspace(-0.2,0.2,7);
Alpha2_R_FD = [0.2 0.2 0.2 0.2 0.2 0.2 0.2];
Alpha3_R_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.01];
Alpha4_R_FD = [0,0,0,0,0,0,0];
Alpha5_R_FD = [0,0,0,0,0,0,0];
Alpha6_R_FD = [0,0,0,0,0,0,0];


Alpha1_L_FD = linspace(-0.2,0.2,7);
Alpha2_L_FD = -[0.2 0.2 0.2 0.2 0.2 0.2 0.2];
Alpha3_L_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.01];
Alpha4_L_FD = [0,0,0,0,0,0,0];
Alpha5_L_FD = [0,0,0,0,0,0,0];
Alpha6_L_FD = [0,0,0,0,0,0,0];

Alpha_R_FD = [Alpha1_R_FD, Alpha2_R_FD, Alpha3_R_FD, Alpha4_R_FD, Alpha5_R_FD, Alpha6_R_FD];
Alpha_L_FD = [Alpha1_L_FD, Alpha2_L_FD, Alpha3_L_FD, Alpha4_L_FD, Alpha5_L_FD, Alpha6_L_FD];
Alpha = [Alpha_R_FD;Alpha_L_FD];
%}
LIP_para.H = 0.7;
LIP_para.T = 0.8;
%H = 0.7;
%%

Alpha1_R_FD = [0.7 0.7 0.7 0.7 0.7 0.7 0.7];
Alpha2_R_FD = [0,0,0,0,0,0,0];
Alpha3_R_FD = [0,0,0,0,0,0,0];
Alpha4_R_FD = [0,0,0,0,0,0,0];
Alpha5_R_FD = linspace(-0.2,0.2,7);
Alpha6_R_FD = [0.2 0.2 0.2 0.2 0.2 0.2 0.2];
Alpha7_R_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.01];
Alpha8_R_FD = [0,0,0,0,0,0,0];
Alpha9_R_FD = [0,0,0,0,0,0,0];
Alpha10_R_FD = [0,0,0,0,0,0,0];

Alpha1_L_FD = [0.7 0.7 0.7 0.7 0.7 0.7 0.7];
Alpha2_L_FD = [0,0,0,0,0,0,0];
Alpha3_L_FD = [0,0,0,0,0,0,0];
Alpha4_L_FD = [0,0,0,0,0,0,0];
Alpha5_L_FD = linspace(-0.2,0.2,7);
Alpha6_L_FD = -[0.2 0.2 0.2 0.2 0.2 0.2 0.2];
Alpha7_L_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.01];
Alpha8_L_FD = [0,0,0,0,0,0,0];
Alpha9_L_FD = [0,0,0,0,0,0,0];
Alpha10_L_FD = [0,0,0,0,0,0,0];

Alpha_R_FD = [Alpha1_R_FD, Alpha2_R_FD, Alpha3_R_FD, Alpha4_R_FD, Alpha5_R_FD, Alpha6_R_FD, Alpha7_R_FD, Alpha8_R_FD, Alpha9_R_FD, Alpha10_R_FD];
Alpha_L_FD = [Alpha1_L_FD, Alpha2_L_FD, Alpha3_L_FD, Alpha4_L_FD, Alpha5_L_FD, Alpha6_L_FD, Alpha7_L_FD, Alpha8_L_FD, Alpha9_L_FD, Alpha10_L_FD];
Alpha = [Alpha_R_FD;Alpha_L_FD];

%{
Alpha1 = 0.2217*ones(1,7);
Alpha2 = [0,0.1,0.15,0.2,0.15,0.1,0];
Alpha3 = zeros(1,7);
Alpha4 = [0 0.005 0.008 0.01 0.008 0.005 0]*2;
Alpha5 = zeros(1,7);
Alpha6 = zeros(1,7);
Alpha_r = [Alpha1,Alpha2,Alpha3,Alpha4,Alpha5,Alpha6];
Alpha_l = Alpha_r;
Alpha = [Alpha_r;Alpha_l];
%}
t=[];
x_sol=[];


t_global=[];

global_position_reference=[];




step=6;
t_end_of_previous_step=0;

t_interrupt=[];
foot_index = -1;
xe_vec = zeros(step,30);

if foot_index == -1
    current_stance_foot_position=forward_kinematics.digit_left_foot_pose(x0(1:30));  %Right foot as stance foot
elseif foot_index == 1
    current_stance_foot_position=forward_kinematics.digit_right_foot_pose(x0(1:30));  %Left foot as stance foot
end
%current_stance_foot_position(1)=0.0376;
%% main loop
for i=1:step
    options=odeset('Events',@(t,x) dynamics.switch_events(t,x,foot_index));

    [t_each_step,x_each_step,te,xe,ie]=ode45(@(t,x) dynamics.dynamics(t,x,...
        foot_index,Alpha,current_stance_foot_position,t_end_of_previous_step,LIP_para),tspan,x0,options);
 
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
    dq_plus=dynamics.resetmap(x_each_step(end,:),foot_index);
    x0=[x_each_step(end,1:30)';dq_plus];
    
    if foot_index==-1
        current_stance_foot_position=forward_kinematics.digit_left_foot_pose(x0(1:30));
        fprintf('heihei')
    elseif foot_index == 1
        current_stance_foot_position=forward_kinematics.digit_right_foot_pose(x0(1:30));
        fprintf('xixi')
    end
    fprintf('hey,Ive finished %d step\n',i)
    stance_foot_position_overall_mat=[stance_foot_position_overall_mat;current_stance_foot_position'];
    foot_index=(-1)^(i+1);

end

%% generate the animation
floating_base_animation2(t,x_sol)
%rmpath('gen')
%% generate the tracking results
figure
hold on
plot(t,x_sol(:,1))
plot(t_global,global_position_reference)
hold off
legend('actual position', 'reference')

%figure
%hold on
%plot(global_position_reference(2:30:end,1),global_position_reference(2:30:end,2),'--')
%plot(stance_foot_position_overall_mat(:,1),stance_foot_position_overall_mat(:,2),'o')
%hold off

