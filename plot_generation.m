%% generate plot
figure
subplot(4,1,1)
title('sagittal')
hold on
plot(t_global, x0_LIP_sagittal_global(1,:))
ylabel('x_{sc}')
%xlim([0 25])
%ylim([-0.02 0.02])
hold off
subplot(4,1,2)
hold on
plot(t_global, x0_LIP_sagittal_global(2,:))
plot(t_global, AM_prediction_global(1,:))
ylabel('L_y')
%ylim([-2 2])
%xlim([0 25])
hold off

subplot(4,1,3)
title('lateral')
hold on
plot(t_global, x0_LIP_lateral_global(1,:))
ylabel('y_{sc}')
%xlim([0 25])
%ylim([-0.2 0.2])
hold off
subplot(4,1,4)
hold on
plot(t_global, x0_LIP_lateral_global(2,:))
plot(t_global, AM_prediction_global(2,:))
ylabel('L_x')
%xlim([0 25])
%ylim([-20 20])
xlabel('time (s)')
hold off
%%
figure
subplot(2,1,1)
hold on
%plot(t_LIP_global,LIP_para.noninitial.sagittal_LIP.u_star*ones(length(t_LIP_global),1),'--')
plot(t_global,hc_global(5,:),'*')
plot(t_global,hd_global(5,:),'o')
plot(t_LIP_global,u_saittal_global)
hold off
legend('hc','hd','planned step length full order model')

%xlim([0 5])
subplot(2,1,2)
hold on

%plot(t_LIP_global,LIP_para.noninitial.lateral_LIP.Left.u_star*ones(length(t_LIP_global),1),'--')
%plot(t_LIP_global,LIP_para.noninitial.lateral_LIP.Right.u_star*ones(length(t_LIP_global),1),'--')
plot(t_global,hc_global(6,:),'*')
plot(t_global,hd_global(6,:),'o')
plot(t_LIP_global,u_lateral_global)
%xlim([0 5])
xlabel('time (s)')
legend('hc','hd','planned step length full order model')
hold off
%% 
figure
subplot(2,1,1)
[t_LIP_global_removed,x0_LIP_sagittal_global_removed] =  Tool.remove_dupliate(t_LIP_global,x0_LIP_sagittal_global);
hold on
plot(t_LIP_global_removed(1:end),46.51*9.81*x0_LIP_sagittal_global_removed(1,:),'Linewidth',3)
plot(t_LIP_global_removed(1:end-1),diff(x0_LIP_sagittal_global_removed(2,:))./diff(t_LIP_global_removed(1:end)),'*')
hold off
legend('mgx_{sc}','dL')
title('sagittal')
subplot(2,1,2)
[t_LIP_global_removed,x0_LIP_lateral_global_removed] =  Tool.remove_dupliate(t_LIP_global,x0_LIP_lateral_global);
%pp = pchip(t_LIP_global_removed,x0_LIP_global_removed);
hold on
plot(t_LIP_global_removed(1:end),-46.51*9.81*x0_LIP_lateral_global_removed(1,:),'Linewidth',3)
plot(t_LIP_global_removed(1:end-1),diff(x0_LIP_lateral_global_removed(2,:))./diff(t_LIP_global_removed(1:end)),'*')
hold off
legend('mgy_{sc}','dL')
title('lateral')

%%
ylabels= {'N','N','N','Nm','Nm','Nm'};
for i = 1:6
    subplot(6,1,i)
    plot(t_global,Fr_global(i,:))
    ylabel(ylabels{i})
end
xlabel('time (s)')

%% generate plot compare with the MuJoCo
load ../LIP_motion_data/Digit/x_sol_desired_sagittal_static_v14
t_mujoco_vec = load("~/Dropbox/UML_dropbox/research/conferenceNjournal_paper/under_actuated_robot_DRS/fromGitHub/digit_mujoco_gym/saved_data/t_abs_np_seq.csv");
x_sc_mujoco_vec = load("~/Dropbox/UML_dropbox/research/conferenceNjournal_paper/under_actuated_robot_DRS/fromGitHub/digit_mujoco_gym/saved_data/x_sc_np_seq.csv");
y_sc_mujoco_vec = load("~/Dropbox/UML_dropbox/research/conferenceNjournal_paper/under_actuated_robot_DRS/fromGitHub/digit_mujoco_gym/saved_data/y_sc_np_seq.csv");
t_mujoco_vec = t_mujoco_vec-0.5;
figure
subplot(4,1,1)
title('sagittal')
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(1,:))
plot(t_vec_desired,x_sol_desired(:,1))
plot(t_mujoco_vec,x_sc_mujoco_vec(:,1))
ylabel('x_{sc}')
xlim([0 0.5])
hold off
legend('full','ALIP','MuJoCo')
subplot(4,1,2)
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(2,:))
plot(t_vec_desired,x_sol_desired(:,2))
plot(t_mujoco_vec,x_sc_mujoco_vec(:,2))
ylabel('L_y')
xlim([0 0.5])
hold off

load ../LIP_motion_data/Digit/x_sol_desired_lateral_static_v14.mat
subplot(4,1,3)
title('lateral')
hold on
plot(t_LIP_global, x0_LIP_lateral_global(1,:))
plot(t_vec_desired,x_sol_desired(:,1))
plot(t_mujoco_vec,y_sc_mujoco_vec(:,1))
ylabel('y_{sc}')
xlim([0 0.5])
hold off
subplot(4,1,4)
hold on
plot(t_LIP_global, x0_LIP_lateral_global(2,:))
plot(t_vec_desired,x_sol_desired(:,2))
plot(t_mujoco_vec,y_sc_mujoco_vec(:,2))
ylabel('L_x')
xlim([0 0.5])
xlabel('time (s)')
hold off
%% data filter
[t_global_filtered, V_global_filtered, ...
    contact_indictor_global_filtered,...
    AM_prediction_global_filtered, x0_LIP_sagittal_global_filtered, ...
    x0_LIP_lateral_global_filtered, AM_COM_global_filtered] = Tool.backwardDataOut();
%% Lyapunov
[V_global_filtered_clean, outlier_indices ]= rmoutliers(V_global_filtered);
t_global_filtered_clean = t_global_filtered(~outlier_indices);
contact_indictor_global_filtered_clean = contact_indictor_global_filtered(~outlier_indices);
plot(t_global_filtered_clean, (V_global_filtered_clean))
Tool.domainIndicator(contact_indictor_global_filtered_clean,t_global_filtered_clean,min(V_global_filtered_clean),max(V_global_filtered_clean))
xlabel('time (s)')
ylabel('V')
%% AM prediction
load ../ALIP_analytical_solution_test/ALIP_DRS_caseC.mat
figure
subplot(4,1,1)
title('sagittal')
hold on
plot(t_global_filtered, x0_LIP_sagittal_global_filtered(1,:))
plot(t_vec_analytical,ALIP_analytical_vec(1,:))
%plot(t_yuan_abs_seq ,xt_yuan_seq(:,1))
ylabel('x_{sc}')
xlim([0 1.7])
ylim([-0.2 0.2])
legend('full-order','ALIP', 'MuJoCo')
hold off
subplot(4,1,2)
title('lateral')
hold on
plot(t_global_filtered, x0_LIP_lateral_global_filtered(1,:))
plot(t_vec_analytical,ALIP_analytical_vec(3,:))
%plot(t_yuan_abs_seq ,yt_yuan_seq(:,1))
ylabel('y_{sc}')
xlim([0 1.7])
ylim([-0.8 0.8])
%xlim([0 8])
%ylim([-0.2 0.2])
hold off


subplot(4,1,3)




hold on
h1 = plot(t_global_filtered, x0_LIP_sagittal_global_filtered(2,:));
h2 = plot(t_vec_analytical,ALIP_analytical_vec(2,:));
%h3 = plot(t_yuan_abs_seq ,xt_yuan_seq(:,2));
h4 = plot(t_global_filtered, AM_prediction_global_filtered(1,:));
ylabel('L_y')
xlim([0 1.7])

%legend('show', 'Location', 'best', 'AutoUpdate', 'off', 'Items', 1:3)

%ylim([-2 2])
%xlim([0 25])
%Tool.domainIndicator(contact_indictor_global_filtered,t_global_filtered,...
%    min(AM_prediction_global_filtered(1,:)),max(AM_prediction_global_filtered(1,:)))
legend([h1,h2,h4],'full-order','ALIP','AM prediction')
hold off
subplot(4,1,4)
hold on
plot(t_global_filtered, x0_LIP_lateral_global_filtered(2,:))
plot(t_vec_analytical,ALIP_analytical_vec(4,:))
%plot(t_global_filtered, AM_prediction_global_filtered(2,:))
%plot(t_yuan_abs_seq ,yt_yuan_seq(:,2))
ylabel('L_x')
xlim([0 1.7])
%xlim([0 25])
%ylim([-20 20])
xlabel('time (s)')

Tool.domainIndicator(contact_indictor_global_filtered,t_global_filtered,...
    min(AM_prediction_global_filtered(2,:)),max(AM_prediction_global_filtered(2,:)))
hold off
%% load MuJoCo
t_yuan_abs_seq = readmatrix("~/Dropbox/UML_dropbox/research/conferenceNjournal_paper/under_actuated_robot_DRS/fromGitHub/digit_mujoco_gym/TRO2023/DRS_int/caseA/t_yuan_abs_seq.csv");
xt_yuan_seq = readmatrix("~/Dropbox/UML_dropbox/research/conferenceNjournal_paper/under_actuated_robot_DRS/fromGitHub/digit_mujoco_gym/TRO2023/DRS_int/caseA/xt_yuan_seq.csv");
yt_yuan_seq = readmatrix("~/Dropbox/UML_dropbox/research/conferenceNjournal_paper/under_actuated_robot_DRS/fromGitHub/digit_mujoco_gym/TRO2023/DRS_int/caseA/yt_yuan_seq.csv");
plot(t_yuan_abs_seq ,xt_yuan_seq )
%% plot base orientation
subplot(2,1,1)

plot(t,x_sol(:,4))
ylabel('rad')
title('roll')
subplot(2,1,2)

plot(t,x_sol(:,5))
ylabel('rad')
title('pitch')
xlabel('time (s)')