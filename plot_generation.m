%% generate plot
figure
subplot(4,1,1)
title('sagittal')
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(1,:))
ylabel('x_{sc}')
%xlim([0 25])
ylim([-0.02 0.02])
hold off
subplot(4,1,2)
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(2,:))
ylabel('L_y')
ylim([-2 2])
%xlim([0 25])
hold off

subplot(4,1,3)
title('lateral')
hold on
plot(t_LIP_global, x0_LIP_lateral_global(1,:))
ylabel('y_{sc}')
%xlim([0 25])
%ylim([-0.2 0.2])
hold off
subplot(4,1,4)
hold on
plot(t_LIP_global, x0_LIP_lateral_global(2,:))
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