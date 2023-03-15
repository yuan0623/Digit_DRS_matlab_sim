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
    hc_global hd_global arm_pose_global Fr_global step_count
tspan=[0 0.95];
addpath('gen')
%addpath("~/Dropbox/UML_dropbox/Matlab_third_party_package")
% set the intitial condition.
%load initial_pose.mat

%%
foot_index = 1;
%%%%%   working data with walking height = 0.71
%load ../LIP_motion_data/Digit/digit_lateral_LIP_v1.mat
%load ../LIP_motion_data/Digit/digit_sagittal_LIP_v1.mat
%%%%%

%%%%%   working data with walking height = 0.71
load ../LIP_motion_data/Digit/digit_lateral_static_LIP_v14.mat
load ../LIP_motion_data/Digit/digit_sagittal_static_LIP_v16.mat
%% 
%%%%%
LIP_para.initial.sagittal_LIP = sagittal_LIP;
LIP_para.initial.lateral_LIP = lateral_LIP;
LIP_para.initial.sagittal_LIP.T = LIP_para.initial.sagittal_LIP.T/2;
LIP_para.initial.lateral_LIP.T = LIP_para.initial.lateral_LIP.T/2;

load ../LIP_motion_data/Digit/digit_lateral_static_LIP_v14.mat
load ../LIP_motion_data/Digit/digit_sagittal_static_LIP_v16.mat
LIP_para.noninitial.sagittal_LIP = sagittal_LIP;
LIP_para.noninitial.lateral_LIP = lateral_LIP;
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
dq0 = [ 0.16472205, -0.07582235, -0.14178435,  0.03851232,  0.05339287,  0.01475982, -0.07628616,...
  0.02661768,  0.11579834, -0.08520979, -0.19279612,  0.49146141,  0.03404071,...
 -0.16216706, -0.00085011,  0.00019614, -0.00004675,  0.00005245, -0.07799193,...
 -0.0181153,  -0.8222329,  -0.72350771,  0.28312948, -0.65882423,  0.2222615,...
 -0.12128032,  0.00248773, -0.00034299,  0.00060361, -0.00023921]';
dq0 = zeros(30,1);
x0 = [q0;dq0];
%{
x0 = [0.04645511 -0.00593438  0.94067885 -0.0502494  -0.04662111  0.01980825...
  0.29703766  0.01068313  0.31047759  0.13742269 -0.05179873 -0.02635923...
  0.03498323 -0.03128759 -0.10349829  0.89771773 -0.00755183  0.36228671...
 -0.31875985  0.03113156 -0.27414325 -0.06783108 -0.00313638  0.07715855...
 -0.06318222  0.07277244  0.10110156 -0.89648244  0.0069389  -0.3625117...
  0.1329908  -0.07278012 -0.07333705  0.08449233 -0.66941043  0.21019579...
  0.16061859  0.16180009  0.64415412 -0.10823709 -0.32921098  0.14193104...
  0.28152266 -0.16663444  0.03621916 -0.00077136  0.00240156  0.00337141...
  0.27925165  0.15452639 -0.10434028 -1.30723239  1.16357474  0.163006...
  1.59569826  6.85909107 -0.02210384 -0.00961818  0.00038172 -0.00267651]';  % MuJoCo IC

q0 = [ 0.04724419  0.00948748  0.972288   -0.01153651 -0.00111326 -0.00211968...
  0.37248052 -0.01440529  0.28128606  0.18253807 -0.01739451 -0.14640034...
 -0.01293713 -0.08774635  0.08643063  0.09114971 -0.00094893 -0.10590642...
  0.89770965 -0.00778789  0.36210181 -0.31736853  0.02864552 -0.27551352...
 -0.18033733  0.01232627  0.15253168  0.01136451  0.1041071  -0.06193412...
 -0.08791674  0.0689317   0.10446422 -0.89582043  0.00696051 -0.3621479 ]';
dq0 = [ 0.00910991  0.0126516  -0.01661561  0.00129456  0.00086217 -0.00007702...
  0.01411391  0.00413885 -0.01298267 -0.04932473  0.00015456  0.04943834...
 -0.00030708  0.01643747 -0.00615679 -0.01210531  0.0164288  -0.00066697...
  0.00136871 -0.0005484   0.00022506  0.01505237  0.00476341  0.01021632...
  0.04712655  0.00008055 -0.04712137 -0.00014423 -0.00363007  0.01435385...
 -0.01492034  0.01754231 -0.00108393  0.00098661 -0.0005519  -0.00005813]';
dq0 = zeros(30,1);
SimpleIdx = [0,1,2,3,4,5,...
                    6,7,8,9,10,11, 15,16,17,18,19,20,...
                    21,22,23,24,25,26, 30,31,32,33,34,35]+1;
q0 = q0(SimpleIdx);
%dq0 = dq0(SimpleIdx);
support_foot = forward_kinematics.digit_left_foot_position(q0);
q0(3) = q0(3)- support_foot(3);
x0 = [q0;dq0];
q0_opt = Tool.obtain_initial_pose_foot_width(q0,foot_index)
%}

%{
%{
x0 = [0.00000016  0.00000022  1.03000018  0.0000004  -0.00000855  0.00386176...
  0.36540167 -0.00540489  0.29889598  0.34498649 -0.01264806 -0.31907522...
  0.1181973  -0.01146249 -0.1057467   0.89666895 -0.00727967  0.36191416...
 -0.33188193  0.00540582 -0.29889571 -0.34498655  0.0126489   0.31897502...
 -0.11819785  0.01130749  0.10573701 -0.89665635  0.00747327 -0.36208294...
  0.00011643  0.00027959  0.00000503  0.00051302 -0.01080443  0.00036644...
  0.00211577 -0.00586886 -0.13474507 -0.4185992   0.4758167  -0.74984716...
  0.68983342  0.04639072  0.00370426 -0.00021655  0.00005399  0.00023812...
 -0.00036035  0.00703839  0.1351806   0.41848568 -0.4746751   0.74982216...
 -0.6903606   0.0056759  -0.00286444  0.0001793  -0.00003324 -0.00024115]';
%}
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
%}
%%

%load data/x0_standing_pose.mat
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
Alpha6_R_FD = (swing_foot(2)-current_stance_foot_position(2))*[1,1,1,1,1,1,1];
Alpha7_R_FD = [0 0.011 0.077 0.1 0.077 0.011 -0.001]/1.5;
Alpha8_R_FD = [0,0,0,0,0,0,0];
Alpha9_R_FD = [0,0,0,0,0,0,0];
Alpha10_R_FD = [0,0,0,0,0,0,0];

Alpha1_L_FD = [1 1 1 1 1 1 1]*lateral_LIP.H;
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

x0_LIP_sagittal_global = [];
x0_LIP_lateral_global = [];
t_LIP_global = [];
x_global = [];
u_saittal_global = [];
u_lateral_global = [];
hc_global = [];
hd_global = [];
Fr_global = [];
step=40;
t_end_of_previous_step=0;

t_interrupt=[];
t_end_desired = 0;
xe_vec = zeros(step,30);

step_count = 1;
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
    
    if step_count == 1
        t_end_desired = LIP_para.initial.sagittal_LIP.T*i;

    else
        t_end_desired = LIP_para.noninitial.sagittal_LIP.T*(i-1)+LIP_para.initial.sagittal_LIP.T;
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
floating_base_animation2(t,x_sol,1,"digit_UA_MuJoCo_x0_v3")
%rmpath('gen')
%% generate the tracking results
figure
hold on
plot(t,x_sol(:,1))
%plot(t_global,global_position_reference)
hold off
%legend('actual position', 'reference')
