function x_star_predict = LIP_run(x0,LIP_para,is_lateral)

    tspan=[0 4];

    
    T = LIP_para.sagittal_LIP.T;

    %% compute the desired foot step
    %u = desired_step_length();

    options=odeset('Events',@(t,x) LIP_planner.switch_event(t,x,T),'RelTol',1e-6,'AbsTol',1e-6);
    %% continuous phase
    [t_each_step,x_each_step,te,~,~]=ode45(@(t,x) LIP_planner.dynamics(t,x,LIP_para,is_lateral),tspan,x0,options);

    x_star_predict = x_each_step(end,:)';
end