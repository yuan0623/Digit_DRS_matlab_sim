function export_frame(robot,x_sol,t_vec,LIP_para,is_DRS_planner , frame_indexs)
    robot_config = homeConfiguration(robot);
    x = [-1 3 3 -1];
    y = [-1 -1 1 1];
    z = [1 1 1 1]*-0.08;
    T_DRS_x = LIP_para.noninitial.T_DRS_x;
    T_DRS_y = LIP_para.noninitial.T_DRS_y;
    amplitude_x = LIP_para.noninitial.amplitude_x;
    amplitude_y = LIP_para.noninitial.amplitude_y;
    %hold on
    terrain = patch(x,y,z,'red');
    
    for i = 1:length(frame_indexs)
        frame_index = frame_indexs(i);
        t = t_vec(frame_index);
        q = x_sol(frame_index,1:30);
        DRS_x = amplitude_x *cos(2*pi/T_DRS_x*t)-amplitude_x;
        DRS_y = amplitude_y *cos(2*pi/T_DRS_y*t)-amplitude_y;
        x = [-1 3 3 -1]+DRS_x ;
        y = [-1 -1 1 1]+DRS_y;
        q_config = assign_each_joint(q);
        for j=1:length(q)
            robot_config(j).JointPosition = q_config(j);
        end
        %hold on
        show(robot,robot_config,'Frames', 'off');
        xlim([-1.5+DRS_x, 3+DRS_x]);
        zlim([-0.1,1.8]);
        ylim([-1.5+DRS_y, 1.5+DRS_y]);
        set(gca,'XTickLabel',[]);
        set(gca,'YTickLabel',[]);
        set(gca,'ZTickLabel',[]);
        set(gcf, 'Position', [10 10 1848 944]);
        % Get rid of tool bar and pulldown menus that are along top of figure.
        set(gcf, 'Toolbar', 'none', 'Menu', 'none');
        %show(robot,robot_config);
        view([44,22])
        %terrain = patch(x,y,z,'red');
        %text1 = text(q(1), q(2), 1.6, [' t = ',sprintf('%.2fs',t)]);
    end
    %hold off
    %drawnow
    if is_DRS_planner
        planner_type = "DRS_int_";
    else 
        planner_type = "Yukai_";
    end
    file_name = "animation_frame/caseC_"+planner_type+string(frame_index)+".png";
    saveas(gcf,file_name)


end




function q_config = assign_each_joint(q)
    q_config = zeros(length(q),1);
    q_config(1:6) = q(1:6);
    q_config(7:14) = q(7:14);
    q_config(15:22) = q(19:26);
    q_config(23:26) = q(15:18);
    q_config(27:30) = q(27:30);
end