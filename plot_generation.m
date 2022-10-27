%% generate plot
load ../LIP_motion_data/x_sol_desired_sagittal_v1

subplot(4,1,1)
title('sagittal')
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(1,:))
plot(t_vec_desired,x_sol_desired(:,1))
xlim([0 2])
hold off
legend('full','ALIP')
subplot(4,1,2)
hold on
plot(t_LIP_global, x0_LIP_sagittal_global(2,:))
plot(t_vec_desired,x_sol_desired(:,2))
xlim([0 2])
hold off

load ../LIP_motion_data/x_sol_desired_lateral_v11.mat
subplot(4,1,3)
title('lateral')
hold on
plot(t_LIP_global, x0_LIP_lateral_global(1,:))
plot(t_vec_desired,x_sol_desired(:,1))
xlim([0 2])
hold off
subplot(4,1,4)
hold on
plot(t_LIP_global, x0_LIP_lateral_global(2,:))
plot(t_vec_desired,x_sol_desired(:,2))
xlim([0 2])
hold off
%%
subplot(2,1,1)
hold on
plot(t_LIP_global,u_saittal_global)
plot(t_LIP_global,LIP_para.sagittal_LIP.u_star*ones(length(t_LIP_global),1),'--')
plot(t_global,hc_global(5,:),'*')
plot(t_global,hd_global(5,:),'o')
hold off
xlim([0 0.6])
subplot(2,1,2)
hold on
plot(t_LIP_global,u_lateral_global)
plot(t_LIP_global,LIP_para.lateral_LIP.Left.u_star*ones(length(t_LIP_global),1),'--')
plot(t_LIP_global,LIP_para.lateral_LIP.Right.u_star*ones(length(t_LIP_global),1),'--')
plot(t_global,hc_global(6,:),'*')
plot(t_global,hd_global(6,:),'o')
xlim([0 0.6])
hold off