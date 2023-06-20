function generateAnimiation(robot,x_sol,t_vec,LIP_para,want_record,name)
    robot_config = homeConfiguration(robot);


    file_path=strcat("video/",name);
    if want_record == 1
        video_obj = VideoWriter(file_path);
        video_obj.getProfiles
        %video_obj.set
        %video_obj.FileFormat = 'mp4';
        open(video_obj)
    end


    x = [-1 1 1 -1];
    y = [-1 -1 1 1];
    z = [1 1 1 1]*-0.08;
    T_DRS_x = LIP_para.noninitial.T_DRS_x;
    T_DRS_y = LIP_para.noninitial.T_DRS_y;
    amplitude_x = LIP_para.noninitial.amplitude_x;
    amplitude_y = LIP_para.noninitial.amplitude_y;
    t_0 = t_vec(1);
    for i=2:length(t_vec)
        t = t_vec(i);
        if t-t_0>5.0000e-03
            t_0 = t;
            q = x_sol(i,1:30);        
            x = [-1 1 1 -1]+amplitude_x *cos(2*pi/T_DRS_x*t)-amplitude_x ;
            y = [-1 -1 1 1]+amplitude_y *cos(2*pi/T_DRS_y*t)-amplitude_y;
            q_config = assign_each_joint(q);
            for j=1:length(q)
                robot_config(j).JointPosition = q_config(j);
            end
            %hold on
            show(robot,robot_config,'Frames', 'off','PreservePlot', false);
            xlim([-1.5+q(1), 1.5+q(1)]);
            zlim([-0.1,1.8]);
            ylim([-1.5+q(2), 1.5+q(2)]);
            set(gca,'XTickLabel',[]);
            set(gca,'YTickLabel',[]);
            set(gca,'ZTickLabel',[]);
            set(gcf, 'Position', [10 10 1840 944]);
            % Get rid of tool bar and pulldown menus that are along top of figure.
            set(gcf, 'Toolbar', 'none', 'Menu', 'none');
            %show(robot,robot_config);
            view([44,22])
            terrain = patch(x,y,z,'red');
            text1 = text(q(1), q(2), 1.6, [' t = ',sprintf('%.2fs',t)]);
            drawnow


            if want_record == 1
                F= getframe(gcf);
                writeVideo(video_obj,F)
            end

            delete(terrain)
            delete(text1)
            %hold off
            i
        end
    end
    if want_record == 1
        close(video_obj)
    end
end




function q_config = assign_each_joint(q)
    q_config = zeros(length(q),1);
    q_config(1:6) = q(1:6);
    q_config(7:14) = q(7:14);
    q_config(15:22) = q(19:26);
    q_config(23:26) = q(15:18);
    q_config(27:30) = q(27:30);
end