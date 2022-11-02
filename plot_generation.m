%% generate plot
load ../LIP_motion_data/Digit/x_sol_desired_sagittal_v1
figure
subplot(4,1,1)
title('sagittal')
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(1,:))
plot(t_vec_desired,x_sol_desired(:,1))
ylabel('x_{sc}')
xlim([0 5])
hold off
legend('full','ALIP')
subplot(4,1,2)
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(2,:))
plot(t_vec_desired,x_sol_desired(:,2))
ylabel('L_y')
xlim([0 5])
hold off

load ../LIP_motion_data/Digit/x_sol_desired_lateral_v1.mat
subplot(4,1,3)
title('lateral')
hold on
plot(t_LIP_global, x0_LIP_lateral_global(1,:))
plot(t_vec_desired,x_sol_desired(:,1))
ylabel('y_{sc}')
xlim([0 5])
hold off
subplot(4,1,4)
hold on
plot(t_LIP_global, x0_LIP_lateral_global(2,:))
plot(t_vec_desired,x_sol_desired(:,2))
ylabel('L_x')
xlim([0 5])
xlabel('time (s)')
hold off
%%
figure
subplot(2,1,1)
hold on
plot(t_LIP_global,u_saittal_global)
plot(t_LIP_global,LIP_para.sagittal_LIP.u_star*ones(length(t_LIP_global),1),'--')
plot(t_global,hc_global(5,:),'*')
plot(t_global,hd_global(5,:),'o')
hold off
legend('planned step length full order model','u_{star}','hc','hd')

xlim([0 5])
subplot(2,1,2)
hold on
plot(t_LIP_global,u_lateral_global)
plot(t_LIP_global,LIP_para.lateral_LIP.Left.u_star*ones(length(t_LIP_global),1),'--')
plot(t_LIP_global,LIP_para.lateral_LIP.Right.u_star*ones(length(t_LIP_global),1),'--')
plot(t_global,hc_global(6,:),'*')
plot(t_global,hd_global(6,:),'o')
xlim([0 5])
xlabel('time (s)')
legend('planned step length full order model','u_{star} left','u_{star} right','hc','hd')
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